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
Bootstrap a new ROS release.
"""

import re
import random
import time
import argparse
import datetime
import shutil
from typing import Optional, Dict
from pathlib import Path
from termcolor import COLORS, HIGHLIGHTS
from yaspin import yaspin
from yaspin.spinners import Spinners
from bazelflore.utils.bzlmod import add_version_to_metadata_json
from bazelflore.utils.bzlmod import regenerate_integrity_hashes
from bazelflore.utils.bzlmod import add_boilerplate_build_file
from bazelflore.utils.bzlmod import increment_version
from bazelflore.utils.bzlmod import scan_module_for_dependencies
from bazelflore.utils.bzlmod import update_package_dependencies

def _check_release_is_valid(working_directory: Path, release: str, overwrite: bool = False) -> Optional[Path]:
    """
    Check if the release already exists.

    Returns True if the release exists and has not yet been patched.
    """
    modules_dir = working_directory / "modules" / "ros"

    release_exists = False
    patch_exists = False
    for x in modules_dir.iterdir():
        if not x.is_dir():
            continue
        if x.name == release:
            release_exists = True
        if x.name.startswith(f"{release}.rcr."):
            patch_exists = True
    if release_exists and (not patch_exists or overwrite):
        return modules_dir / release
    return None

def _find_candidate_patch_version(package_dir: Path) -> Optional[str]:
    """
    Find the most recent patch version for a given package version.

    This function searches the package_dir for all module versions matching the format
    <release>.rcr.<x>. If none exist, then it returns None. Otherwise, it sorts the
    version candidates by <release> and then <x> and selects the last candidate. In
    other words the most recent patch from the latest release.
    """
    if not package_dir.exists():
        return None
    candidates = []
    for item in package_dir.iterdir():
        if not item.is_dir():
            continue
        version = item.name
        if '.rcr.' in version:
            version_parts = version.split('.')
            patch_str = ".".join(version_parts[:-2])
            patch_num = int(version_parts[-1])
            candidates.append((patch_str, patch_num))
    if not candidates:
        return None

    # Sort by patch number
    candidates.sort()

    # Return the name of the last candidate (highest patch number)
    return "{0}.rcr.{1}".format(candidates[-1][0], candidates[-1][1])


def _package_migrate(
    package_dir: Path,                          # Package base path.
    package_new_version: str,                   # Destination version.
    package_ref_version: str,                   # For the MODULE.bazel and source.json.
    package_old_version: Optional[str]):        # For the patches and overlays.
    """
    Create a new package from package_ref_version
    """
    # Make sure we have a new folder for the package.
    (package_dir / package_new_version).mkdir(parents=True, exist_ok=True)

    # Add the version to metadata.json
    add_version_to_metadata_json(package_dir / "metadata.json", package_new_version)

    # Copy the MODULE.bazel from the "reference version"
    for copyfile in ['MODULE.bazel', 'source.json']:
        shutil.copy(
            package_dir / package_ref_version / copyfile,
            package_dir / package_new_version / copyfile)

    # If this is the first time we've ever seen this package, create a boilerplate BUILD file.
    if package_old_version is not None:
        for copydir in ['patches', 'overlay']:
            if (package_dir / package_old_version / copydir).exists():
                shutil.copytree(
                    package_dir / package_old_version / copydir,
                    package_dir / package_new_version / copydir)

    # Copy the MODULE.bazel and source.json from the "reference version". This is to handle
    # the case where the "old version" is the last release, which may have stale dependencies.
    for copyfile in ['MODULE.bazel', 'source.json']:
        ref_file = package_dir / package_ref_version / copyfile
        new_file = package_dir / package_new_version / copyfile
        if ref_file.exists():
            new_file.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(ref_file, new_file)

    # Regenerate the integrity hashes and return the new version.
    regenerate_integrity_hashes(package_dir / package_new_version)
    return package_new_version

def main():

    # Gather arguments
    parser = argparse.ArgumentParser(description="Migrate an existing RCR release to a new release.")
    parser.add_argument('working_directory', type=Path)
    parser.add_argument(
        "--release",
        required=True,
        help="Bazel 'ros' module release version, e.g. rolling.2026-01-21",
    )
    parser.add_argument(
        "--from-distro",
        help="Migrate from a distro other than the one in the release string",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite the workspace directory if it exists",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't actually create the new patch release, just print what we would do",
    )
    args = parser.parse_args()
    new_release = f'{args.release}.rcr.0'

    with yaspin(text="Migrating release {0} to {1}".format(args.release, new_release), color="cyan") as sp:
        # REF: rolling.2026-01-21.        (bare release without patches)
        # OLD: rolling.2026-01-21.bcr.2   (what to start with)
        # NEW: rolling.2026-01-21.bcr.3   (what we are adding)
        modules_dir = args.working_directory / "modules"

        sp.write("> Ensuring REF release is valid")
        ref_dir = _check_release_is_valid(args.working_directory, args.release, args.overwrite)
        if ref_dir is None:
            raise RuntimeError("Release {0} exists or already has patches.".format(args.release))
        sp.write("> Release exists and there are no patches, continuing")

        sp.write("> Scanning REF release for packages")
        ref_packages = scan_module_for_dependencies(ref_dir / 'MODULE.bazel', modules_dir)
        ref_packages["ros"] = args.release
        sp.write("> Found {0} packages in REF release".format(len(ref_packages)))

        # Look for old
        sp.write("> Migrating modules")
        new_packages = {}
        for package_name, package_ref_version in ref_packages.items():
            if '.rcr.' in package_ref_version:
                sp.write("  + Skipping {0}@{1}".format(package_name, package_ref_version))
                continue
            package_dir = modules_dir / package_name

            # Determine the new version.
            package_new_version = increment_version(package_ref_version)

            # Get the most up to date package version.
            package_old_version = _find_candidate_patch_version(package_dir)
    
            # If we're not doing a dry run, actually migrate the package.
            if not args.dry_run:
                package_new_version = _package_migrate(
                    package_dir,
                    package_new_version,
                    package_ref_version,
                    package_old_version
                )

            # Inform what happened
            if package_old_version is not None:
                sp.write("  + Migrated {0}@{1} -> {0}@{2}".format(
                    package_name, package_old_version, package_new_version))
            else:
                sp.write("  + Created {0}@{1}".format(package_name, package_new_version))

            # Log the new package name.
            new_packages[package_name] = package_new_version

        # Create a new release dir for the new packages.
        new_dir = args.working_directory / 'modules' / 'ros' / f'{args.release}.rcr.0'
        sp.write("> Added NEW release {0}".format(new_dir.name))

        # Run through all MODULE.bazel files replacing any bazel_dep calls to old
        # package versions with the new package versions.
        sp.write("> Updating dependencies")
        for package_name, package_new_version in new_packages.items():
            module_file = args.working_directory / "modules" / package_name / package_new_version / "MODULE.bazel"
            if not args.dry_run:
                update_package_dependencies(module_file, package_name, ref_packages, new_packages)
            sp.write("  + Updated dependencies for {0}@{1}".format(package_name, package_new_version))

        sp.ok("✔")


if __name__ == "__main__":
    main()