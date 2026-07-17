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
import sys
from pathlib import Path

def parse_diff_status_file(file_path: Path) -> list[tuple[str, str]]:
    diffs = []
    try:
        with open(file_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split("\t", 1)
                if len(parts) == 2:
                    diffs.append((parts[0], parts[1]))
        return diffs
    except Exception as e:
        print(f"Error reading diff status file {file_path}: {e}", file=sys.stderr)
        sys.exit(1)

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

        # Rule 3: Only metadata.json files can be modified under target_dir
        if status.startswith("M"):
            path_parts = Path(file_path).parts
            is_metadata = (
                len(path_parts) == len(target_parts) + 2 and
                path_parts[:len(target_parts)] == target_parts and
                path_parts[-1] == "metadata.json"
            )
            
            if not is_metadata:
                violations.append(f"Modified existing file (only metadata.json files can be modified): {file_path}")
                continue

            # For modified metadata.json, check version rules
            meta_violations = check_metadata_json(file_path, old_metadata_dir, working_dir)
            violations.extend(meta_violations)
            
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

    diffs = parse_diff_status_file(args.diff_status)
    violations = check_violations(diffs, args.old_metadata_dir, Path("."), args.directory)

    if violations:
        print(f"PR violations found in '{args.directory}' directory:")
        for violation in violations:
            print(f" - {violation}")
        sys.exit(1)

    print(f"No violations found in '{args.directory}' directory.")
    sys.exit(0)

if __name__ == "__main__":
    main()
