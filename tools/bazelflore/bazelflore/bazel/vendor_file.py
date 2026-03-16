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
from bazelflore.bazel.constants import ROS_TO_BAZEL_MAPPING
from bazelflore.bazel.module import Module
from bazelflore.sources.bcr import BcrSource
from bazelflore.sources.deb import DebSource
from bazelflore.sources.ros import RosSource


class VendorFile:
    def __init__(self, working_directory: Path):
        """Initialize the module."""
        self.module_dir = working_directory / 'modules'
        self.vendor_file = working_directory / 'vendor' / 'VENDOR.bazel'

    def generate_file(self):
        """Generate the module."""

        ret = get_copyright_header()
        # TODO(asymingt): scan the self.module_dir for bazel_dep() calls. If the
        # name of the module is self.module_dir then pin this module. Otherwise,
        # it must be a BCR module and we should ignore() ot.

        # Write the vendor file.
        self.vendor_file.parent.mkdir(parents=True, exist_ok=True)
        with open(self.vendor_file, 'w') as f:
            f.write(ret)
