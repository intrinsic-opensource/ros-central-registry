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
Remove a previously-applied patch set from the RCR.
"""

import argparse
import datetime
import shutil
from pathlib import Path
from termcolor import COLORS, HIGHLIGHTS
from typing import Optional, Dict, List
from yaspin import yaspin
from yaspin.spinners import Spinners
from bazelflore.utils.bzlmod import increment_version
from bazelflore.utils.bzlmod import find_latest_patch
from bazelflore.utils.bzlmod import find_release
from bazelflore.utils.bzlmod import scan_module_for_dependencies
from bazelflore.utils.bzlmod import get_module_diff
from bazelflore.utils.bzlmod import load_patches_and_overlays
from bazelflore.utils.bzlmod import regenerate_integrity_hashes
from bazelflore.utils.bzlmod import add_version_to_metadata_json
from bazelflore.utils.bzlmod import update_package_dependencies
from bazelflore.utils.bzlmod import remove_version_from_metadata_json   

def _get_all_previous_patch_versions(working_directory: Path, release: str) -> Optional[List[str]]:
    """
    Check if the given version string is a valid patch version. This means that
    it must end in .rcr.X, where X is a non-negative integer.
    """
    version_parts = release.split(".")
    if len(version_parts) < 2:
        return None
    version_num = int(version_parts[-1])
    if version_parts[-2] != 'rcr' or version_num < 0:
        return None
    if not (working_directory / "modules" / "ros" / release).exists():
        return None
    release_template = ".".join(version_parts[:-1]) + ".{version}"
    return [
        release_template.format(version=i)
        for i in range(0, version_num)
        if (working_directory / "modules" / "ros" / release_template.format(version=i)).exists()
    ]

def main():

    # Gather arguments
    parser = argparse.ArgumentParser(description="Generate a RCR patch from a given release workspace")
    parser.add_argument('working_directory', type=Path)
    parser.add_argument(
        "--workspace",
        required=True,
        type=str,
        help="Bazel 'ros' module release version, e.g. rolling.2026-01-21",
    )
    args = parser.parse_args()

    with yaspin(text="Unpatching modules folder", color="cyan") as sp:
        # REF: rolling.2026-01-21.        (bare release without patches)
        # OLD: rolling.2026-01-21,bcr.2   (what to start with)
        # NEW: rolling.2026-01-21.bcr.3   (what we are adding)
        modules_dir = args.working_directory / "modules"

        # Make sure the patch version is valid and it exists.
        prev_versions = _get_all_previous_patch_versions(args.working_directory, args.workspace)
        if prev_versions is None:
            raise RuntimeError("Patch version {0} does not exist".format(args.workspace))
        sp.write("> Found previous patch versions: {0}".format(prev_versions))

        # Find all modules used by this version.
        new_module_file = args.working_directory / "modules" / "ros" / args.workspace / 'MODULE.bazel'
        new_packages = scan_module_for_dependencies(new_module_file, modules_dir)
        new_packages["ros"] = args.workspace
        sp.write("> Found {0} packages".format(len(new_packages)))

        # Make sure we don't remove any modules needed by previous releases.
        for prev_version in prev_versions:
            prev_module_file = args.working_directory / "modules" / "ros" / prev_version / 'MODULE.bazel'
            prev_packages = scan_module_for_dependencies(prev_module_file, modules_dir)
            prev_packages["ros"] = prev_version
            for package_name, prev_package_version in prev_packages.items():
                if package_name in new_packages:
                    next_package_version = new_packages[package_name]
                    if next_package_version == prev_package_version:
                        del new_packages[package_name]
        sp.write("> Found {0} packages to prune".format(len(new_packages)))

        # Remove all the old package versions.
        for package_name in sorted(new_packages.keys()):
            package_dir = args.working_directory / "modules" / package_name
            package_version = new_packages[package_name]
            shutil.rmtree(package_dir / package_version, ignore_errors=True)
            remove_version_from_metadata_json(package_dir / "metadata.json", package_version)
            sp.write("> Removed {0} version {1}".format(package_name, package_version))
        sp.ok("✔")


if __name__ == "__main__":
    main()