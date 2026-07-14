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
Generic module creator for the "rosdistro" module.
"""

from pathlib import Path
from typing import Dict
from bazelflore.bazel.module import Module
from bazelflore.sources.bcr import BcrSource
from bazelflore.sources.deb import DebSource
from bazelflore.sources.ros import RosSource


class RosdistroModule(Module):
    def __init__(self,
                 working_directory: Path,
                 bcr_sources: Dict[str, BcrSource],
                 deb_sources: Dict[str, DebSource],
                 ros_sources: Dict[str, RosSource],
                 release_distro: str,
                 release_date: str):
        """Initialize the module."""
        super().__init__(
            working_directory=working_directory,
            bcr_sources=bcr_sources,
            deb_sources=deb_sources,
            ros_sources=ros_sources,
            release_distro=release_distro,
            release_date=release_date,
            module_name="rosdistro",
            module_version="{0}.{1}".format(release_distro, release_date),
            module_url="https://github.com/asymingt/rosdistro/archive/refs/tags/{0}/{1}.tar.gz".format(
                release_distro, release_date),
            package_version="{0}.{1}".format(release_distro, release_date)
        )

        # The module content for rosdistro does not need any dependencies. In
        # fact it pins dependencies for other packages in ROS.
        self.custom_module_bazel = """
# Third party deps
bazel_dep(name = "assimp", version = "6.0.3")
bazel_dep(name = "boost.endian", version = "1.90.0.bcr.1")
bazel_dep(name = "bullet", version = "3.26.0-rc0.bcr.2")
bazel_dep(name = "console_bridge", version = "1.0.1")
bazel_dep(name = "cppcheck", version = "2.20.0")
bazel_dep(name = "cpplint", version = "2.0.2")
bazel_dep(name = "curl", version = "8.12.0")
bazel_dep(name = "cyclonedds", version = "0.10.5")
bazel_dep(name = "eigen", version = "5.0.1.bcr.1")
bazel_dep(name = "fastcdr", version = "2.3.5.bcr.0")
bazel_dep(name = "fastdds", version = "3.4.2.bcr.1")
bazel_dep(name = "gz-common", version = "7.1.1")
bazel_dep(name = "gz-fuel-tools", version = "11.0.0")
bazel_dep(name = "gz-math", version = "9.1.0")
bazel_dep(name = "gz-msgs", version = "12.0.1")
bazel_dep(name = "gz-physics", version = "9.2.0")
bazel_dep(name = "gz-plugin", version = "4.0.0")
bazel_dep(name = "gz-rendering", version = "10.0.1")
bazel_dep(name = "gz-sensors", version = "10.0.1")
bazel_dep(name = "gz-sim", version = "10.2.0")
bazel_dep(name = "gz-transport", version = "15.0.2")
bazel_dep(name = "gz-utils", version = "4.0.0")
bazel_dep(name = "libyaml", version = "0.2.5")
bazel_dep(name = "lz4", version = "1.9.4")
bazel_dep(name = "mcap", version = "1.4.0")
bazel_dep(name = "mimick", version = "0.9.0")
bazel_dep(name = "ogre", version = "1.12.10")
bazel_dep(name = "opencv", version = "4.13.0.bcr.5")
bazel_dep(name = "orocos-kdl", version = "1.5.3")
bazel_dep(name = "pybind11_bazel", version = "3.0.1")
bazel_dep(name = "sdl2", version = "2.32.0.bcr.beta.5")
bazel_dep(name = "spdlog", version = "1.17.0")
bazel_dep(name = "sqlite3", version = "3.51.2.bcr.1")
bazel_dep(name = "tinyxml2", version = "10.0.0")
bazel_dep(name = "uncrustify", version = "0.82.0")
bazel_dep(name = "yaml-cpp", version = "0.9.0")
bazel_dep(name = "zenoh-cpp", version = "1.8.0")
bazel_dep(name = "zlib", version = "1.3.1.bcr.5")
bazel_dep(name = "zstd", version = "1.5.6")

# Setup python
python = use_extension("@rules_python//python/extensions:python.bzl", "python")
python.toolchain(
    is_default = True,
    python_version = "3.12",
)

# Setup pip
pip = use_extension("@rules_python//python/extensions:pip.bzl", "pip")
pip.parse(
    hub_name = "pip_ros",
    python_version = "3.12",
    requirements_lock = "requirements.txt",
)
use_repo(pip, "pip_ros")

# Set up llvm for clang-format and clang-tidy
llvm = use_extension("@llvm//extensions:llvm.bzl", "llvm")
use_repo(llvm, "llvm-project")

# Setup qt
qt = use_extension("@rules_qt//extension:qt.bzl", "fetch")
qt.install(
    name = "qt_linux_x86_64",
    build_file = "@rules_qt//extension:qt/6.8.3/linux_x86_64.BUILD",
    os = "linux",
    version = "6.8.3",
)
use_repo(qt, "qt_linux_x86_64")
register_toolchains("@rules_qt//tools:all")
"""
