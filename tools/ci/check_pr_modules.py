#!/usr/bin/env python3
# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import json
import os
import re
import sys
from pathlib import Path

from tools.ci.bzlmod_lib import increment_version, parse_diff_status_file

def check_metadata_json(file_path: str, old_metadata_dir: Path, working_dir: Path) -> list[str]:
    violations = []
    
    # Read the old metadata.json from old_metadata_dir
    old_metadata_path = old_metadata_dir / file_path
    if not old_metadata_path.exists():
        violations.append(f"Old metadata file {file_path} not found in {old_metadata_dir}.")
        return violations
        
    try:
        with open(old_metadata_path, "r") as f:
            old_metadata = json.load(f)
    except Exception as e:
        violations.append(f"Failed to parse old metadata JSON in {file_path}: {e}")
        return violations

    # Read the new metadata.json from working tree
    new_path = working_dir / file_path
    if not new_path.exists():
        violations.append(f"Modified file {file_path} does not exist in the working directory.")
        return violations
        
    try:
        with open(new_path, "r") as f:
            new_metadata = json.load(f)
    except Exception as e:
        violations.append(f"New {file_path} is not valid JSON: {e}")
        return violations

    # Verify versions are not removed without being yanked
    old_versions = old_metadata.get("versions", [])
    new_versions = new_metadata.get("versions", [])
    new_yanked = new_metadata.get("yanked_versions", {})

    if not isinstance(old_versions, list):
        violations.append(f"Old {file_path} 'versions' field must be a list.")
        return violations
    if not isinstance(new_versions, list):
        violations.append(f"New {file_path} 'versions' field must be a list.")
        return violations
    if not isinstance(new_yanked, dict):
        violations.append(f"New {file_path} 'yanked_versions' field must be a dictionary.")
        return violations

    for version in old_versions:
        if version not in new_versions and version not in new_yanked:
            violations.append(
                f"Version '{version}' was removed from 'versions' in {file_path} but was not added to 'yanked_versions'."
            )

    return violations

def check_module_bazel(file_path: str, old_metadata_dir: Path, working_dir: Path, package_name: str) -> list[str]:
    violations = []

    # If it is the rosdistro module, we allow any edits.
    if package_name == "rosdistro":
        return violations

    # Otherwise, check the only change is the version argument bumping the patch suffix by one.
    old_module_path = old_metadata_dir / file_path
    if not old_module_path.exists():
        violations.append(f"Old MODULE.bazel file {file_path} not found in {old_metadata_dir}.")
        return violations

    new_module_path = working_dir / file_path
    if not new_module_path.exists():
        violations.append(f"Modified file {file_path} does not exist in the working directory.")
        return violations

    try:
        with open(old_module_path, "r") as f:
            old_content = f.read()
    except Exception as e:
        violations.append(f"Failed to read old MODULE.bazel in {file_path}: {e}")
        return violations

    try:
        with open(new_module_path, "r") as f:
            new_content = f.read()
    except Exception as e:
        violations.append(f"Failed to read new MODULE.bazel in {file_path}: {e}")
        return violations

    pattern = re.compile(
        r'module\(\s*name\s*=\s*["\']([^"\']+)["\']\s*,\s*version\s*=\s*["\']([^"\']+)["\']'
    )

    old_match = pattern.search(old_content)
    if not old_match:
        violations.append(f"Could not find module declaration for '{package_name}' in old MODULE.bazel {file_path}")
        return violations

    old_name, old_version = old_match.groups()
    if old_name != package_name:
        violations.append(f"Module name mismatch in old MODULE.bazel: expected '{package_name}', found '{old_name}'")
        return violations

    new_match = pattern.search(new_content)
    if not new_match:
        violations.append(f"Could not find module declaration for '{package_name}' in new MODULE.bazel {file_path}")
        return violations

    new_name, new_version = new_match.groups()
    if new_name != package_name:
        violations.append(f"Module name mismatch in new MODULE.bazel: expected '{package_name}', found '{new_name}'")
        return violations

    try:
        expected_version = increment_version(old_version)
    except ValueError as e:
        violations.append(f"Failed to increment version '{old_version}': {e}")
        return violations

    if new_version != expected_version:
        violations.append(
            f"MODULE.bazel version was changed from '{old_version}' to '{new_version}', "
            f"but it should have been incremented to '{expected_version}'."
        )
        return violations

    # Replace the new version string back to the old version string in new_content
    normalized_new_content = (
        new_content[:new_match.start(2)] + old_version + new_content[new_match.end(2):]
    )

    if normalized_new_content != old_content:
        violations.append(f"MODULE.bazel has modifications other than the version bump: {file_path}")

    return violations

def check_violations(diffs: list[tuple[str, str]], old_metadata_dir: Path, working_dir: Path, target_dir: str) -> list[str]:
    violations = []
    
    # Ensure target_dir has a trailing slash for prefix matching (e.g. "modules/" or "bcr_staging/modules/")
    prefix = target_dir.strip("/") + "/"
    target_parts = Path(prefix).parts

    for status, file_path in diffs:
        # We only care about files under target_dir
        if not file_path.startswith(prefix):
            continue

        # Rule 1: A PR cannot delete any files under target_dir
        if status.startswith("D"):
            violations.append(f"Deleted file in {prefix}: {file_path}")
            continue

        # Rule 2: A PR cannot rename files under target_dir
        if status.startswith("R"):
            violations.append(f"Renamed file in {prefix}: {file_path}")
            continue

        # Rule 3: Only metadata.json and MODULE.bazel files can be modified under target_dir
        if status.startswith("M"):
            path_parts = Path(file_path).parts
            is_metadata = (
                len(path_parts) == len(target_parts) + 2 and
                path_parts[:len(target_parts)] == target_parts and
                path_parts[-1] == "metadata.json"
            )
            
            is_module_bazel = (
                len(path_parts) == len(target_parts) + 3 and
                path_parts[:len(target_parts)] == target_parts and
                path_parts[-1] == "MODULE.bazel"
            )

            if is_metadata:
                meta_violations = check_metadata_json(file_path, old_metadata_dir, working_dir)
                violations.extend(meta_violations)
            elif is_module_bazel:
                package_name = path_parts[len(target_parts)]
                module_violations = check_module_bazel(file_path, old_metadata_dir, working_dir, package_name)
                violations.extend(module_violations)
            else:
                violations.append(f"Modified existing file (only metadata.json and MODULE.bazel files can be modified): {file_path}")
                continue

    return violations

def main():
    parser = argparse.ArgumentParser(description="Check PR changes for modules rules.")
    parser.add_argument(
        "--diff-status",
        required=True,
        type=Path,
        help="Path to the file containing git diff --name-status output"
    )
    parser.add_argument(
        "--old-metadata-dir",
        required=True,
        type=Path,
        help="Path to the directory containing the old metadata.json files from base branch"
    )
    parser.add_argument(
        "--directory",
        default="modules",
        help="Directory to check (e.g. modules or bcr_staging/modules)"
    )
    args = parser.parse_args()

    # `bazel run`'s child process cwd is the exec root, not the actual git
    # checkout -- BUILD_WORKSPACE_DIRECTORY is what recovers the real path
    # (see the same pattern in setup_workspace.py/vendor_modules.py/
    # create_patch.py). Without this, every modified metadata.json under
    # `working_dir` spuriously looks "missing".
    workspace_root = Path(os.environ.get("BUILD_WORKSPACE_DIRECTORY", ".")).resolve()

    diffs = parse_diff_status_file(args.diff_status)
    violations = check_violations(diffs, args.old_metadata_dir, workspace_root, args.directory)

    if violations:
        print(f"PR violations found in '{args.directory}' directory:")
        for violation in violations:
            print(f" - {violation}")
        sys.exit(1)

    print(f"No violations found in '{args.directory}' directory.")
    sys.exit(0)

if __name__ == "__main__":
    main()
