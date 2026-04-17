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

import random
import time
import argparse
import datetime
from pathlib import Path
from typing import Optional
from termcolor import COLORS, HIGHLIGHTS
from yaspin import yaspin
from yaspin.spinners import Spinners
from bazelflore.bazel.package_module import PackageModule
from bazelflore.bazel.ros_module import RosModule
from bazelflore.bazel.rosdistro_module import RosdistroModule
from bazelflore.sources.bcr import BcrWorker
from bazelflore.sources.deb import DebWorker, COMPONENTS
from bazelflore.sources.ros import RosWorker

# These are packages that we'd like a user to be able to import by release
# date rather than version. It means that we'll have new package versions
# for these every time that we cut a release.
DATE_VERSIONED_PACKAGES = [
    "rosdistro",
    "ros",
    "ros_core",
    "ros_base",
    "desktop",
    "desktop_full",
    "simulation",
    "perception"
]

def _get_version_for_package(name, version, release_distro, release_date):
    if name in DATE_VERSIONED_PACKAGES:
        return "{0}.{1}".format(release_distro, release_date)
    return version

def _find_previous_package_release(working_directory: Path, pkg_name: str, pkg_version: str) -> Optional[str]:
    """
    Find a the most recent patched version of the given package.
    """
    package_dir = working_directory / "modules" / pkg_name
    if not package_dir.exists():
        return None

    # Iterate over subfolders in package_dir, expecting version numbers
    # of the form <version>[.rcr.<patch_number>]. If no folder exists
    # with a <version> equal to the pkg_version argument, return None
    # to signal that this version is not yet in the RCR. If this is not
    # the case, find and return highest patch number of this version.
    final_version = None
    final_integer = None
    for path in package_dir.iterdir():
        if not path.is_dir():
            continue
        name = path.name
        if name == pkg_version and final_integer is None:
            final_version = name
            final_integer = -1
        elif name.startswith(pkg_version + ".rcr."):
            patch_integer = int(name.split('.')[-1])
            if final_integer is None or patch_integer > final_integer:
                final_version = name
                final_integer = patch_integer
    return final_version


def _valid_date(s: str) -> str:
    try:
        datetime.datetime.strptime(s, "%Y-%m-%d")
        return s
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a valid date: {s!r}")

def main():
    parser = argparse.ArgumentParser(description="Bootstrap a new RCR release.")
    parser.add_argument('working_directory', type=Path)
    parser.add_argument(
        "--ros-release-distro",
        required=True,
        choices=["jazzy", "kilted", "lyrical", "rolling"],
        help="ROS distro, e.g. lyrical",
    )
    parser.add_argument(
        "--ros-release-date",
        required=True,
        type=_valid_date,
        help="ROS release, e.g. 2026-01-21",
    )
    parser.add_argument(
        "--ubuntu-distro",
        required=True,
        choices=["noble", "resolute"],
        help="Ubuntu distro, e.g. noble",
    )
    parser.add_argument(
        "--ubuntu-architecture",
        required=True,
        help="Snapshot architecture, e.g. amd64",
    )
    args = parser.parse_args()

    # Execute on the task
    release_version = "{0}.{1}".format(args.ubuntu_distro, args.ros_release_date)

    with yaspin(text="Bootstrapping new release", color="cyan") as sp:
    
        sp.write("> Setting up ubuntu worker")
        deb_worker = DebWorker(args.working_directory, args.ubuntu_distro, args.ubuntu_architecture, args.ros_release_date)
        deb_sources = deb_worker.fetch_packages()
        sp.write("> Collected info about {0} debians".format(len(deb_sources)))

        sp.write("> Scanning BCR for latest module versions")
        bcr_worker = BcrWorker(args.working_directory)
        bcr_sources = bcr_worker.fetch_packages()
        sp.write("> Collected info about {0} modules".format(len(bcr_sources)))

        sp.write("> Setting up rosdistro snapshot")
        ros_worker = RosWorker(args.working_directory, args.ubuntu_distro, args.ros_release_distro, args.ros_release_date)
        ros_sources = ros_worker.fetch_packages()
        sp.write("> Collected info about {0} packages".format(len(ros_sources)))

        sp.write("> Calculating which modules already exist in the RCR")
        resolved_packages = dict()
        for pkg_name in sorted(ros_sources.keys()):

            # Determine the final version for the package. We have to do this because
            # some packages, like rosdistro, have a release date version in RCR,
            pkg_version = _get_version_for_package(
                pkg_name,
                ros_sources[pkg_name].version,
                args.ros_release_distro,
                args.ros_release_date
            )

            # Find the latest patched release in RCR that has the same distro
            previous_release_version = _find_previous_package_release(
                args.working_directory, pkg_name, pkg_version)

            # Print what we find and log any package versions that exist.
            if previous_release_version is None:
                print("+ {0}@{1} is new".format(pkg_name, pkg_version))
                resolved_packages[pkg_name] = (False, pkg_version)
            else:
                print("+ {0}@{1} is old)".format(pkg_name, previous_release_version))
                resolved_packages[pkg_name] = (True, previous_release_version)

        sp.write("> Creating Bazel modules for all ROS packages")
        for pkg_name in sorted(ros_sources.keys()):
            if resolved_packages[pkg_name][0]:
                continue
            sp.write("> Processing {0}".format(pkg_name))
            package_module = PackageModule(
                working_directory=args.working_directory,
                bcr_sources=bcr_sources,
                deb_sources=deb_sources,
                ros_sources=ros_sources,
                release_distro=args.ros_release_distro,
                release_date=args.ros_release_date,
                module_name=pkg_name,
                module_version=resolved_packages[pkg_name][1],
                module_url=ros_sources[pkg_name].url,
            )
            for dep_name in ros_sources[pkg_name].dependencies:
                if dep_name in ros_sources.keys():
                    package_module.add_dependency(dep_name, resolved_packages[dep_name][1])
            package_module.generate_module()
        sp.write("> Created package bazel modules")

        sp.write("> Creating rosdistro bazel module")
        rosdistro_module = RosdistroModule(
            working_directory=args.working_directory,
            bcr_sources=bcr_sources,
            deb_sources=deb_sources,
            ros_sources=ros_sources,
            release_distro=args.ros_release_distro,
            release_date=args.ros_release_date,
        )
        rosdistro_module.generate_module()
        sp.write("> Created rosdistro bazel module")
    
        sp.write("> Creating ros bazel module")
        ros_module = RosModule(
            working_directory=args.working_directory,
            bcr_sources=bcr_sources,
            deb_sources=deb_sources,
            ros_sources=ros_sources,
            release_distro=args.ros_release_distro,
            release_date=args.ros_release_date,
        )
        for dep_name in ros_sources.keys():
            ros_module.add_dependency(dep_name, resolved_packages[dep_name][1])
        ros_module.generate_module()
        sp.write("> Created ros bazel module")

        sp.ok("✔")


if __name__ == "__main__":
    main()