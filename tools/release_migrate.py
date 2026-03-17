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

def _check_release_is_valid(working_directory: Path, release: str) -> Optional[Path]:
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
    if release_exists and not patch_exists:
        return modules_dir / release
    return None

def _find_previous_release(working_directory: Path, release: str, distro: Optional[str] = None) -> Optional[Path]:
    """
    Find a directory to the previous release directory.

    All releases are named <distribution>.<date>[.rcr.<patch_number>]. If there is
    no suffix, it is assumed to be the first unpatched release. The goal of this
    code is to find the most recent patched release that is not the current release
    from which to copy patches. For example, if the release string supplied to
    this function is rolling.2026-01-21, and we have the following release set...

        rolling.2025-11-21
        rolling.2025-11-21.rcr.0
        rolling.2025-11-21.rcr.1
        rolling.2025-11-21.rcr.2
        rolling.2025-12-21
        rolling.2026-01-21

    ... the function should return the directory of "rolling.2025-11-21.rcr.2".
    This is because it is the most recent patched release that is from an earlier
    date than the current release. Note that "rolling.2025-12-21" is not chosen
    because it is not patched.
    """
    modules_dir = working_directory / "modules" / "ros"

    # Extract the distro and date from the distribution.
    release_parts = release.split(".")
    if len(release_parts) != 2:
        raise RuntimeError("{0} is not in the format <distribution>.<date>".format(release))
    needle_distro, needle_date_str = release_parts
    needle_date = datetime.datetime.strptime(needle_date_str, "%Y-%m-%d")

    # If no specified distro was supplied, assume it is the same as the release
    if distro is not None:
        needle_distro = distro

    # Find all candidate patch releases.
    candidate_patch_releases = []
    for x in modules_dir.iterdir():
        if not x.is_dir():
            continue
        release = x.name
        release_parts = release.split(".")
        if len(release_parts) != 4:
            continue
        haystack_distro, haystack_date, _, haystack_patch = release_parts
        haystack_date = datetime.datetime.strptime(haystack_date, "%Y-%m-%d")
        if haystack_distro == needle_distro and haystack_date < needle_date:
            candidate_patch_releases.append((haystack_date, int(haystack_patch), release))

    # We have no candidates, so there is no previous release. This should only
    # ever happen when we run this for the very first time for a distro.
    if not candidate_patch_releases:
        return None

    # Sort by date descending, then patch version descending
    candidate_patch_releases.sort(key=lambda x: (x[0], x[1]), reverse=True)

    # Return the release directory that was chosen.
    return candidate_patch_releases[0][2]

def _package_migrate(package_dir: Path, package_base_version: str, package_old_version: str):
    """
    Migrate a package from package_base_version -> package_old_version
    """
    package_new_version = increment_version(package_old_version)

    # Add the version to metadata.json
    add_version_to_metadata_json(package_dir / "metadata.json", package_new_version)
    
    # Create the new version directory
    (package_dir / package_new_version).mkdir(parents=True, exist_ok=True)

    # Copy the MODULE.bazel from the "base version"
    for copyfile in ['MODULE.bazel', 'source.json']:
        shutil.copy(
            package_dir / package_base_version / copyfile,
            package_dir / package_new_version / copyfile)

    # Copy patches and overlay directories from the "old version"
    for copydir in ['patches', 'overlay']:
        if (package_dir / package_old_version / copydir).exists():
            shutil.copytree(
                package_dir / package_old_version / copydir,
                package_dir / package_new_version / copydir)

    # Regenerate the integrity hashes
    return package_new_version

def _package_create(package_dir: Path, package_base_version: str):
    """
    Create a new package from package_base_version
    """
    package_new_version = increment_version(package_base_version)

    # Add the version to metadata.json
    add_version_to_metadata_json(package_dir / "metadata.json", package_new_version)

    # Create the new version directory
    (package_dir / package_new_version / 'overlay').mkdir(parents=True, exist_ok=True)

    # Copy the MODULE.bazel from the "base version"
    for copyfile in ['MODULE.bazel', 'source.json']:
        shutil.copy(
            package_dir / package_base_version / copyfile,
            package_dir / package_new_version / copyfile)

    # Add a boilerplate BUILD file.
    add_boilerplate_build_file(package_dir / package_new_version / 'overlay')

    # Regenerate the integrity hashes
    return package_new_version

def _update_package_dependencies(module_file: Path, package_name, old_packages: Dict[str, str], new_packages: Dict[str, str]):
    """
    Update the bazel_dep() calls in a package's MODULE.bazel file to point to new deps.
    """

    with open(module_file, 'r') as f:
        content = f.read()
    
    old_module_definition = """
module(
    name = "{0}",
    version = "{1}",
    bazel_compatibility = [">=7.2.1"],
)
""".format(package_name, old_packages[package_name])

    new_module_definition = """
module(
    name = "{0}",
    version = "{1}",
    bazel_compatibility = [">=7.2.1"],
)
""".format(package_name, new_packages[package_name])

    content = content.replace(old_module_definition, new_module_definition)
    for package_name in new_packages.keys():
        content = content.replace(
            'bazel_dep(name = "{0}", version = "{1}")'.format(
                package_name, old_packages[package_name]),
            'bazel_dep(name = "{0}", version = "{1}")'.format(
                package_name, new_packages[package_name]))

    with open(module_file, 'w') as f:
        f.write(content)

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
    args = parser.parse_args()
    new_release = f'{args.release}.rcr.0'

    with yaspin(text="Migrating release {0} to {1}".format(args.release, new_release), color="cyan") as sp:

        sp.write("> Ensuring target release is valid")
        base_dir = _check_release_is_valid(args.working_directory, args.release)
        if base_dir is None:
            raise RuntimeError("Release {0} exists or already has patches.".format(args.release))
        sp.write("> Release exists and there are no patches, continuing")

        new_dir = args.working_directory / 'modules' / 'ros' / f'{args.release}.rcr.0'
        sp.write("> New patch release will be cut with name {0}".format(new_dir.name))

        sp.write("> Scanning target release for packages")
        base_packages = scan_module_for_dependencies(base_dir)
        base_packages["ros"] = args.release
        sp.write("> Found {0} packages".format(len(base_packages)))

        sp.write("> Checking for previous patch release")
        maybe_old_dir = _find_previous_release(args.working_directory, args.release, args.from_distro)
        prev_packages = {}
        if maybe_old_dir is None:   
            sp.write("> No suitable previous patch release found")
        else:
            sp.write("> Found previous patch release: {0}".format(maybe_old_dir.name))
            sp.write("> Scanning previous patch release for packages")
            prev_packages = scan_module_for_dependencies(maybe_old_dir)
            prev_packages["ros"] = maybe_old_dir.name
            sp.write("> Found {0} packages".format(len(prev_packages)))

        # As we migrate packages, we need to keep track of the old and new versions
        # to update the dependencies after the migration has occured.
        sp.write("> Migrating modules")
        old_packages = {}
        new_packages = {}
        for package_name, package_base_version in base_packages.items():
            sp.write("> Processing {0}".format(package_name))
            package_dir = args.working_directory / "modules" / package_name
            if package_name in prev_packages.keys():
                package_new_version = _package_migrate(
                    package_dir,
                    package_base_version,
                    prev_packages[package_name]
                )
                sp.write("  + Migrated {0}@{1} -> {0}@{2}".format(
                    package_name, prev_packages[package_name], package_new_version))
                old_packages[package_name] = prev_packages[package_name]
            else:
                package_new_version = _package_create(
                    package_dir,
                    package_base_version
                )
                sp.write("  + Created new package {0}@{1}".format(
                    package_name, package_new_version))
                old_packages[package_name] = package_base_version
            regenerate_integrity_hashes(package_dir / package_new_version)
            sp.write("  + Regenerated integrity hashes for {0}@{1}".format(
                package_name, package_new_version))
            new_packages[package_name] = package_new_version


        # Run through all MODULE.bazel files replacing any bazel_dep calls to old
        # package versions with the new package versions.
        sp.write("> Updating dependencies")
        for package_name, package_new_version in new_packages.items():
            module_file = args.working_directory / "modules" / package_name / package_new_version / "MODULE.bazel"
            _update_package_dependencies(module_file, package_name, old_packages, new_packages)
            sp.write("  + Updated dependencies for {0}@{1}".format(
                package_name, package_new_version))

        sp.ok("✔")


if __name__ == "__main__":
    main()