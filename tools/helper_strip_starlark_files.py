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

"""
Strips out all existing Starlark code to avoid destructive interference.

This tool iterates through subdirectories of a specified directory, locating
Bazel build files (.bzl, .bazel, WORKSPACE, BUILD).

Usage:
    python helper_strip_starlark_files.py <directory> --workspace <workspace>

Examples:
    python helper_strip_starlark_files.py $PWD --workspace rolling.2026-01-21.bcr.6
"""

import argparse
from datetime import date
from typing import Tuple
from pathlib import Path

def get_copyright_header():
    return """# Copyright {0} Open Source Robotics Foundation, Inc.
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
# limitations under the License.\n
""".format(date.today().year)

def process_directory(working_directory : Path, workspace : str) -> Tuple[int, int]:
    """
    Process a directory tree, removing all Starlark code, and returns stats
    about how many files were removed from how many packages.

    Returns:
        A tuple of (file_count, package_count).
    """
    base_dir = working_directory / 'workspace' / workspace / 'vendor'
    if not base_dir.exists():
        print(f"Error: '{base_dir}' is not a directory.")
        return 0, 0

    file_count = 0
    package_count = 0
    for candidate in sorted(base_dir.iterdir()):
        internal_dir = candidate.name.startswith("_") or candidate.name.startswith("bazel-") 
        if internal_dir or not candidate.is_dir():
            continue
        removals = 0
        for wildcard in ["**/*.bzl", "**/BUILD", "**/WORKSPACE", "**/*.bazel"]:
            for path in (base_dir / candidate).rglob(wildcard):
                if path.name != "MODULE.bazel":
                    path.unlink()
                    removals += 1
        if removals > 0:
            print(f"Removed {removals} Starlark file(s) from {candidate.name}")
            package_count += 1
        with open(candidate / "BUILD.bazel", "w") as f:
            f.write(get_copyright_header())
        file_count += removals
    return file_count, package_count


def main():
    """Entry point for the interface_autogen tool."""
    parser = argparse.ArgumentParser(
        description=(
            "Iterate through all packages in a workspace and strip out all existing"
            "Starlark code to avoid destructive interference with the RCR."
        ),
    )   
    parser.add_argument(
        "working_directory",
        help="Root directory to scan for interface files.",
        type=Path,
    )
    parser.add_argument(
        "--workspace",
        type=str,
        required=True,
        help=(
            "Optional list of specific subdirectories to process. "
            "If not specified, all subdirectories are processed "
            "(except those starting with '_' or 'bazel-')."
        ),
    )
    args = parser.parse_args()
    file_count, package_count = process_directory(args.working_directory, args.workspace)
    print(f"Removed {file_count} Starlark file(s) from {package_count} packages.")

if __name__ == "__main__":
    main()
