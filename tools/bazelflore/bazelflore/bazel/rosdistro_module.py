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
from bazelflore.utils.copyright import get_copyright_header
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
bazel_dep(name = "console_bridge", version = "1.0.1")
bazel_dep(name = "curl", version = "8.12.0")
bazel_dep(name = "lz4", version = "1.9.4")
bazel_dep(name = "mcap", version = "1.4.0")
bazel_dep(name = "opencv", version = "4.13.0.bcr.5")
bazel_dep(name = "protobuf", version = "33.4")
bazel_dep(name = "sdl2", version = "2.32.0.bcr.beta.5")
bazel_dep(name = "sqlite3", version = "3.51.2.bcr.1")
bazel_dep(name = "tinyxml2", version = "10.0.0")
bazel_dep(name = "yaml-cpp", version = "0.9.0")
bazel_dep(name = "zlib", version = "1.3.1.bcr.5")
bazel_dep(name = "zstd", version = "1.5.6")

# Setup Python toolchains and pip repositories for the ROS distro

python = use_extension("@rules_python//python/extensions:python.bzl", "python")
python.toolchain(
    is_default = True,
    python_version = "3.12",
)

pip = use_extension("@rules_python//python/extensions:pip.bzl", "pip")
pip.parse(
    hub_name = "pip_ros",
    python_version = "3.12",
    requirements_lock = "requirements.txt",
)
use_repo(pip, "pip_ros")
"""
