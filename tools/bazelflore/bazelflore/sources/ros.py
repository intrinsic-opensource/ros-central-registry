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
import tempfile
from collections import namedtuple
from pathlib import Path
from typing import Dict, List
from rosinstall_generator.distro import get_distro
from rosinstall_generator.distro import get_package_names
from rosdistro.dependency_walker import DependencyWalker
import rosdep2.sources_list
import rosdep2.lookup
import rosdep2.rospkg_loader
import rosdep2.platforms.debian

RosSource = namedtuple('RosSource', ['version', 'url', 'dependencies'])

# These are packages that I can't seem to resolve to apt packages. So, I'm going to
# skip them for now and follow up on them later with @cottsay.
FORBIDDEN_DEPS = [
    'python3-catkin-pkg-modules',
    'python3-vcstool',
    'python3-rospkg-modules',
    'python3-rosdistro-modules',
    'ros_ign_bridge',
    'ros_ign_gazebo',
    'action_tutorials_interfaces',
    'gazebo_ros_pkgs',
    'pybind11_json_vendor',
    'actionlib_msgs',
]

class RosWorker:
    def __init__(self, working_directory: Path, ubuntu_distro: str, ros_distro: str, ros_release_date: str):
        """Initialize the module."""
        self.working_directory = working_directory
        self.ros_distro = ros_distro
        self.ros_release_date = ros_release_date
        self.ubuntu_distro = ubuntu_distro

        # This is equivalent to running `rosdep init` and `rosdep update` in Ubuntu to create a local
        # cache of rosdep data. We need this to resolve ros keys to apt packages.
        cache_dir = working_directory / ".cache" / "rosdep"
        sources_list_dir = cache_dir / "sources.list.d"
        sources_list_dir.mkdir(parents=True, exist_ok=True)
        with open(os.path.join(sources_list_dir, '20-default.list'), 'w') as f:
            f.write('yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx\n')
            f.write('yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml\n')
            f.write('yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml\n')
            f.write('yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml\n')
            f.write('gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte\n')
        rosdep2.sources_list.update_sources_list(
            sources_list_dir=sources_list_dir,
            sources_cache_dir=cache_dir,
            ros_distro=ros_distro,
            quiet=True,
        )
        matcher = rosdep2.sources_list.DataSourceMatcher(
            ['ubuntu', self.ubuntu_distro, self.ros_distro]
        )
        sources_loader = rosdep2.sources_list.SourcesListLoader.create_default(
            sources_cache_dir=cache_dir,
            matcher=matcher,
        )
        lookup = rosdep2.lookup.RosdepLookup.create_from_rospkg(sources_loader=sources_loader)

        # Build the rosdep lookup view once, outside the per-package loop.
        self.view = lookup.get_rosdep_view(
            rosdep2.rospkg_loader.DEFAULT_VIEW_KEY)
        self.apt_installer = rosdep2.platforms.debian.AptInstaller()

        # Inject a different ROSDISTRO_INDEX_URL to force rosinstall_generator to get the correct
        # release information.  See: https://github.com/ros/rosdistro/pull/50112
        url = "https://github.com/asymingt/rosdistro/releases/download/{0}/{1}/index-v4.yaml".format(
            ros_distro, ros_release_date)
        os.environ["ROSDISTRO_INDEX_URL"] = url

        # Get a list of rosdistro packages and create a dependency walker.
        self.rosdistro = get_distro(ros_distro)
        self.package_names = set(get_package_names(self.rosdistro)[0])
        self.walker = DependencyWalker(self.rosdistro, evaluate_condition_context={
            'ROS_DISTRO': ros_distro,
            'ROS_VERSION': '2',
            'ROS_PYTHON_VERSION': '3'
        })

    def fetch_packages(self) -> Dict[str, RosSource]:
        """
        Fetch all packages in the distro and return a dictionary of packages.
        """

        packages: Dict[str, RosSource] = {}
        
        for pkg_name in self.package_names:

            # Get information about the release package.
            release_package = self.rosdistro.release_packages[pkg_name]

            # Get the release repository name.
            repo = self.rosdistro.repositories[release_package.repository_name].release_repository

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
            pkg_deps = sorted(list(
                set(exec_deps)
                | set(build_deps)
                | set(buildtool_deps)
                | set(build_export_deps)
                | set(buildtool_export_deps)
                | set(run_deps)
                | set(test_deps)
            ))

            # Transform ROS keys to apt debian package names for the target distro.
            os_deps: Set[str] = set()
            for ros_key in pkg_deps:
                if ros_key in self.package_names:
                    os_deps.add(ros_key)
                elif ros_key not in FORBIDDEN_DEPS:
                    try:
                        installer_key, rule = self.view.lookup(ros_key).get_rule_for_platform(
                            'ubuntu', self.ubuntu_distro, ['apt'], 'apt')
                        for os_dep in self.apt_installer.resolve(rule):
                            os_deps.add(os_dep)
                    except Exception as e:
                        print(f"Warning: could not resolve rosdep key '{ros_key}': {e}")

            # Package metadata.
            packages[pkg_name] = RosSource(
                pkg_version,
                pkg_url,
                os_deps
            )

        return packages
