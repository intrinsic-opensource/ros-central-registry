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
Generic module creator for bazel.
"""

import json
import base64
import hashlib
import urllib
from datetime import date
from pathlib import Path
from typing import Dict
from bazelflore.sources.bcr import BcrSource
from bazelflore.sources.ros import RosSource
from bazelflore.sources.deb import DebSource
from bazelflore.bazel.constants import *

class Module:
    """
    Generic Bazel module creator for ROS releases.
    """
    def __init__(self,
                 working_directory: Path,
                 bcr_sources: Dict[str, BcrSource],
                 deb_sources: Dict[str, DebSource],
                 ros_sources: Dict[str, RosSource],
                 release_distro: str,
                 release_date: str,
                 module_name: str,
                 module_version: str,
                 module_url : str,
                 package_version: str):
        """
        Initialize the module.
        """
        
        # This is the version of the generated package.
        self.package_version = package_version

        # Destination directory structure.
        self.cache_dir = working_directory / ".cache"
        self.pkg_dir = working_directory / "modules" / module_name
        self.version_dir = self.pkg_dir / package_version
        self.patches_dir = self.version_dir / "patches"
        self.overlay_dir = self.version_dir / "overlay"
        self.module_file_path = self.version_dir / 'MODULE.bazel'
        self.source_json_path = self.version_dir / 'source.json'
        self.metadata_json_path = self.pkg_dir / 'metadata.json'

        # Release information
        self.release_distro = release_distro
        self.release_date = release_date

        # Module information
        self.module_name = module_name
        self.module_version = module_version
        self.module_url = module_url

        # Dependency information.
        self.bcr_sources = bcr_sources
        self.deb_sources = deb_sources
        self.ros_sources = ros_sources

        # Transformed dependency information.
        self.bcr_deps = {}
        self.rcr_deps = {}

        # Overlay and patch information.
        self.overlays = {}
        self.patches = {}

    def get_copyright_header(self):
        return """# Copyright {0} Open Source Robotics Foundation, Inc.
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
# limitations under the License.\n
""".format(date.today().year)

    def _calculate_sha256_from_file(self, file_path):
        """Calculate a bazel sha256 hash from a file."""
        sha256_hash = hashlib.sha256()
        with open(file_path, "rb") as f:
            for byte_block in iter(lambda: f.read(4096), b""):
                sha256_hash.update(byte_block)
        return "sha256-" + base64.b64encode(sha256_hash.digest()).decode()

    def _generate_strip_prefix(self):
        """
        Generates the strip_prefix for the current Bazel module.
        """
        return "{0}-release-upstream-{1}".format(self.module_name, self.module_version)

    def _download_package_tarball(self):
        """
        Downloads the tarball of a specific commit from a GitHub repository.
        """
        cache_file = self.cache_dir / f"{self.module_name}-{self.module_version}.gz"
        if not cache_file.exists():
            try:
                with urllib.request.urlopen(self.module_url, timeout=60) as resp:
                    data = resp.read()
                    if data:
                        self.cache_dir.mkdir(parents=True, exist_ok=True)
                        cache_file.write_bytes(data)
            except urllib.error.HTTPError as exc:
                raise Exception("Failed to download tarball: {} :: {}".format(self.module_url, exc))
        return cache_file

    def _generate_integrity(self):
        """
        Generates the integrity hash for the current Bazel module.
        """
        tarball = self._download_package_tarball()
        integrity =  self._calculate_sha256_from_file(tarball)
        return integrity


    def _generate_overlay(self):
        """
        Generate the overlays
        """
        overlay = {}
        for rel_path, content in self.overlays.items():
            overlay_path = self.overlay_dir / rel_path
            overlay_path.parent.mkdir(parents=True, exist_ok=True)
            with open(overlay_path, 'w') as f:
                f.write(content)
            overlay[rel_path] = self._calculate_sha256_from_file(overlay_path)
        return overlay

    def _generate_patches(self):
        """
        Generate the patches
        """
        patches = {}
        for rel_path, content in self.patches.items():
            patch_path = self.patches_dir / rel_path
            patch_path.parent.mkdir(parents=True, exist_ok=True)
            with open(patch_path, 'w') as f:
                f.write(content)
            patches[rel_path] = self._calculate_sha256_from_file(patch_path)
        return patches

    def _write_source_json(self):
        """
        Generates the source.json contents for the current Bazel module.
        """
        source_json = {
            "integrity": self._generate_integrity(),
            "url": self.module_url,
            "strip_prefix": self._generate_strip_prefix(),
        }
        if self.overlays:
            source_json["overlay"] = self._generate_overlay()
        if self.patches:
            source_json["patches"] = self._generate_patches()
            source_json["patch_strip"] = 1

        self.source_json_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.source_json_path, 'w') as f:
            json.dump(source_json, f, indent=4)
            f.write('\n')

    def _write_metadata_json(self):
        """
        Update the metadata.json file for the current Bazel module version, if needed.
        """
        metadata = {
            "homepage": self.module_url,
            "maintainers": [
                {
                    "email": "simmers@intrinsic.ai",
                    "github": "asymingt",
                    "github_user_id": 37671,
                    "name": "Andrew Symington"
                }
            ],
            "versions": [],
            "yanked_versions": {}
        }
        
        if self.metadata_json_path.exists():
            with open(self.metadata_json_path, 'r') as f:
                try:
                    metadata = json.load(f)
                except:
                    pass

        if self.package_version not in metadata["versions"]:
            metadata["versions"].append(self.package_version)
            metadata["versions"].sort()

        self.metadata_json_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.metadata_json_path, 'w') as f:
            json.dump(metadata, f, indent=4)
            f.write('\n')

    def _write_module_dot_bazel(self):
        """
        Generates the MODULE.bazel contents for the current Bazel module.
        """
        ret = self.get_copyright_header()
        ret += "# ROS package information\n"
        ret += 'module(\n'
        ret += '    name = "{0}",\n'.format(self.module_name)
        ret += '    version = "{0}.{1}",\n'.format(self.release_distro, self.module_version)
        ret += '    bazel_compatibility = [">=7.2.1"],\n'
        ret += ')\n'
        if self.bcr_deps:
            ret += '\n# BCR dependencies\n'
            for dep in sorted(self.bcr_deps.keys()):
                ret += 'bazel_dep(name = "{0}", version = "{1}")\n'.format(
                    dep, self.bcr_deps[dep])
        if self.rcr_deps:
            ret += '\n# RCR Dependencies\n'
            for dep in sorted(self.rcr_deps.keys()):
                ret += 'bazel_dep(name = "{0}", version = "{1}.{2}")\n'.format(
                    dep, self.release_distro, self.rcr_deps[dep])

        self.module_file_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.module_file_path, 'w') as f:
            f.write(ret)

    def add_dependency(self, name: str):
        """
        Add a dependency to the current Bazel module. The input name is
        a rosdep key, and the logic in this function decides whether it
        should be mapped to a BCR module, a RCR module, or be omitted
        entirely. It also resolves the correction version for the dep.
        """

        # The rosdep key may map to ne or more BCR modules. If this
        # ends up being empty, then we assume the dep is RCR.
        bcr_names = []

        # Test a bunch of different conditions to determine what to do.
        if name in ROS_TO_BAZEL_MAPPING:
            if ROS_TO_BAZEL_MAPPING[name] is None:
                # This means that we intentionally ignore this dep
                return
            bcr_names.extend(ROS_TO_BAZEL_MAPPING[name])
        elif name.startswith("libboost-") \
            or name.startswith("boost"):
            bcr_names.extend(BOOST_DEPS)
        elif name.startswith("libqt5") \
            or name.startswith("qt5") \
            or name.startswith("qtbase5") \
            or name.startswith("qttools5") \
            or name.startswith("qtdeclarative5") \
            or name.startswith("qtmultimedia5") \
            or name.startswith("pyqt5"):
            bcr_names.extend(QT5_DEPS)
        elif name.startswith("libqt6") \
            or name.startswith("qt6") \
            or name.startswith("qtbase6") \
            or name.startswith("qttools6") \
            or name.startswith("qtdeclarative6") \
            or name.startswith("qtmultimedia6") \
            or name.startswith("pyqt6"):
            bcr_names.extend(QT6_DEPS)
        elif name.startswith("pcl") \
            or name.startswith("libpcl"):
            bcr_names.extend(PCL_DEPS)
        elif name.startswith("python3-") \
            or name.startswith("python-"):
            bcr_names.extend(PYTHON_DEPS)

        # First, check to see if we should be adding BCR deps.
        if bcr_names:
            for bcr_name in bcr_names:
                # TODO(asymingt) - make this a bit more robust by using
                # MVS to resolve the nearest version.
                self.bcr_deps[bcr_name] = self.bcr_sources[bcr_name].versions[-1]
        # Second, if this is a ROS package, add it as a dependency.
        elif name in self.ros_sources:
            self.rcr_deps[name] = "{0}".format(self.ros_sources[name].version)
        # If we get there, then there's been a problem.
        else:
            print(f"Unknown key: {name}")

    def generate_module(self):
        """
        Generate the module.
        """
        self._write_module_dot_bazel()
        self._write_source_json()
        self._write_metadata_json()
