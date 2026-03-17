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
from typing import Dict, Set
import re
from bazelflore.bazel.constants import ROS_TO_BAZEL_MAPPING, get_copyright_header
from bazelflore.bazel.module import Module
from bazelflore.sources.bcr import BcrSource
from bazelflore.sources.deb import DebSource
from bazelflore.sources.ros import RosSource


class VendorFile:
    def __init__(self, working_directory: Path):
        """Initialize the module."""
        self.module_dir = working_directory / 'modules'
        self.bazelrc_file = working_directory / 'vendor' / '.bazelrc'
        self.vendor_file = working_directory / 'vendor' / 'VENDOR.bazel'

    def _write_vendor_dot_bazel(self, bcr_deps : Set[str], rcr_deps : Set[str]):
        ret = get_copyright_header()
        for dep in sorted(bcr_deps):
            ret += f'ignore("@@{dep}+")\n'
        for dep in sorted(rcr_deps):
            ret += f'pin("@@{dep}+")\n'
        self.vendor_file.parent.mkdir(parents=True, exist_ok=True)
        with open(self.vendor_file, 'w') as f:
            f.write(ret)

    def _write_dot_bazelrc(self, rcr_deps : Set[str]):
        ret = get_copyright_header()
        ret += "vendor --vendor_dir=vendor \\\n"
        for rcr_dep in sorted(rcr_deps):
            ret += f'--repo=@{rcr_dep} \\\n'
        with open(self.bazelrc_file, 'w') as f:
            f.write(ret)

    def generate_file(self):
        """Generate the module."""

        deps = set()
        for module_file in self.module_dir.rglob('MODULE.bazel'):
            with open(module_file, 'r') as f:
                content = f.read()
                matches = re.findall(r'bazel_dep\(\s*name\s*=\s*"([^"]+)"', content)
                deps.update(matches)
        bcr_deps = set()
        rcr_deps = set()
        for dep in sorted(deps):
            if (self.module_dir / dep).is_dir():
                rcr_deps.add(dep)
            else:
                bcr_deps.add(dep)

        self._write_vendor_dot_bazel(bcr_deps, rcr_deps)
        self._write_dot_bazelrc(rcr_deps)
