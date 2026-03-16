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
Generic module creator for the "ros" module.
"""

import os
from collections import namedtuple
from pathlib import Path
from typing import Dict, Tuple, List
from rosinstall_generator.distro import get_distro
from rosinstall_generator.distro import get_package_names
from rosdistro.dependency_walker import DependencyWalker
from rosdistro.rosdistro import RosPackage

RosSource = namedtuple('RosSource', ['version', 'url', 'dependencies'])

class RosWorker:
    def __init__(self, working_directory: Path, ros_distro: str, ros_release_date: str):
        """Initialize the module."""

        # Inject a different ROSDISTRO_INDEX_URL to force rosinstall_generator to get the correct
        # release information.  See: https://github.com/ros/rosdistro/pull/50112
        url = "https://github.com/asymingt/rosdistro/releases/download/{0}/{1}/index-v4.yaml".format(
            ros_distro, ros_release_date)
        os.environ["ROSDISTRO_INDEX_URL"] = url

        self.distro = get_distro(ros_distro)
        self.walker = DependencyWalker(self.distro, evaluate_condition_context={
            'ROS_DISTRO': ros_distro,
            'ROS_VERSION': '2',
            'ROS_PYTHON_VERSION': '3'
        })

    def fetch_packages(self) -> Dict[str, RosSource]:   
        """
        Fetch all packages in the distro and return a dictionary of packages.
        """

        packages: Dict[str, RosSource] = {}

        for pkg_name in get_package_names(self.distro)[0]:
            
            # Get information about the release package.
            release_package = self.distro.release_packages[pkg_name]
            
            # Get the release repository name.
            repo = self.distro.repositories[release_package.repository_name].release_repository

            # Package version and URL.
            pkg_version = repo.version.split("-")[0]
            pkg_url = '{url}/archive/refs/tags/{tag}.tar.gz'.format(
                url=repo.url.replace('.git', ''),
                tag='upstream/{0}'.format(pkg_version)
            )

            # Get dependencies for various package.xml properties.
            exec_deps = self.walker.get_depends(pkg_name, "exec")
            build_deps = self.walker.get_depends(pkg_name, "build")
            buildtool_deps = self.walker.get_depends(pkg_name, "buildtool")
            build_export_deps = self.walker.get_depends(pkg_name, "build_export")
            buildtool_export_deps = self.walker.get_depends(pkg_name, "buildtool_export")
            run_deps = self.walker.get_depends(pkg_name, "run")
            test_deps = self.walker.get_depends(pkg_name, "test")

            # Aggregate package dependencies.
            pkg_deps = sorted(list(set(exec_deps) \
                 | set(build_deps) \
                 | set(buildtool_deps) \
                 | set(build_export_deps) \
                 | set(buildtool_export_deps) \
                 | set(run_deps) \
                 | set(test_deps)))

            # Package metadata.
            packages[pkg_name] = RosSource(
                pkg_version,
                pkg_url,
                pkg_deps
            )

        return packages
