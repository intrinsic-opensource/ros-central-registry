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

from pathlib import Path
from typing import Dict
from bazelflore.bazel.module import Module
from bazelflore.sources.bcr import BcrSource
from bazelflore.sources.deb import DebSource
from bazelflore.sources.ros import RosSource


class RosModule(Module):
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
            module_name="ros",
            module_version="{0}.{1}".format(release_distro, release_date),
            module_url="https://github.com/ros2/ros2/archive/refs/tags/{0}.tar.gz".format(release_distro)
        )