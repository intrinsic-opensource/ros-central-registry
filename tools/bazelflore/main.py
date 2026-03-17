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
from termcolor import COLORS, HIGHLIGHTS
from yaspin import yaspin
from yaspin.spinners import Spinners
from bazelflore.bazel.constants import ROS_TO_BAZEL_MAPPING
from bazelflore.bazel.package_module import PackageModule
from bazelflore.bazel.ros_module import RosModule
from bazelflore.bazel.rosdistro_module import RosdistroModule
from bazelflore.bazel.vendor_file import VendorFile
from bazelflore.sources.bcr import BcrWorker
from bazelflore.sources.deb import DebWorker, COMPONENTS
from bazelflore.sources.ros import RosWorker


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
        # sp.write("> Setting up ubuntu worker")
        # deb_worker = DebWorker(args.working_directory, args.ubuntu_distro, args.ubuntu_architecture, args.ros_release_date)
        # deb_sources = {}
        # for component in COMPONENTS:
        #     sp.write("> Fetching ubuntu component: {0}".format(component))
        #     deb_sources.update(deb_worker.fetch_packages(component))
        # sp.write("> Collected info about {0} debians".format(len(deb_sources)))

        # sp.write("> Scanning BCR for latest module versions")
        # bcr_worker = BcrWorker(args.working_directory)
        # bcr_sources = bcr_worker.fetch_packages()
        # sp.write("> Collected info about {0} modules".format(len(bcr_sources)))

        # sp.write("> Setting up rosdistro snapshot")
        # ros_worker = RosWorker(args.working_directory, args.ros_release_distro, args.ros_release_date)
        # ros_sources = ros_worker.fetch_packages()
        # sp.write("> Collected info about {0} packages".format(len(ros_sources)))

        # sp.write("> Creating Bazel modules for all ROS packages")
        # for pkg_name in sorted(ros_sources.keys()):
        #     # We intentionally choose to skip some vendored packages in the BCR:
        #     if pkg_name in ROS_TO_BAZEL_MAPPING and ROS_TO_BAZEL_MAPPING[pkg_name] is None:
        #         sp.write("> Skipping vendored package {0}".format(pkg_name))
        #         continue
        #     # If we get here, we need to generate a package module for the current package.
        #     pkg_info = ros_sources[pkg_name]
        #     sp.write("> Processing {0}".format(pkg_name))
        #     package_module = PackageModule(
        #         working_directory=args.working_directory,
        #         bcr_sources=bcr_sources,
        #         deb_sources=deb_sources,
        #         ros_sources=ros_sources,
        #         release_distro=args.ros_release_distro,
        #         release_date=args.ros_release_date,
        #         module_name=pkg_name,
        #         module_version=pkg_info.version,
        #         module_url=pkg_info.url,
        #     )
        #     for dep in pkg_info.dependencies:
        #         package_module.add_dependency(dep)
        #     package_module.generate_module()
        # sp.write("> Created package bazel modules")

        # sp.write("> Creating rosdistro bazel module")
        # rosdistro_module = RosdistroModule(
        #     working_directory=args.working_directory,
        #     bcr_sources=bcr_sources,
        #     deb_sources=deb_sources,
        #     ros_sources=ros_sources,
        #     release_distro=args.ros_release_distro,
        #     release_date=args.ros_release_date,
        # )
        # rosdistro_module.generate_module()
        # sp.write("> Created rosdistro bazel module")
    
        # sp.write("> Creating ros bazel module")
        # ros_module = RosModule(
        #     working_directory=args.working_directory,
        #     bcr_sources=bcr_sources,
        #     deb_sources=deb_sources,
        #     ros_sources=ros_sources,
        #     release_distro=args.ros_release_distro,
        #     release_date=args.ros_release_date,
        # )
        # ros_module.generate_module()
        # sp.write("> Created ros bazel module")

        sp.write("> Creating vendor file")
        vendor_file = VendorFile(working_directory=args.working_directory)
        vendor_file.generate_file()
        sp.write("> Created vendor file")

        sp.ok("✔")


if __name__ == "__main__":
    main()