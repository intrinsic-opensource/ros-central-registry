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
import re
import base64
import hashlib
import tarfile
import urllib
from pathlib import Path
from typing import Dict, List, Tuple
from bazelflore.utils.copyright import get_copyright_header
from bazelflore.utils.bzlmod import add_version_to_metadata_json
from bazelflore.utils.bzlmod import calculate_integrity_hash_for_file
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
        self.bcr_sources = bcr_sources.copy()
        self.deb_sources = deb_sources.copy()
        self.ros_sources = ros_sources.copy()

        # Transformed dependency information.
        self.bcr_deps = DEPS_GENERAL.copy()
        self.rcr_deps = {}

        # Overlay and patch information.
        self.overlays = {}
        self.patches = {}

        # Custom MODULE.bazel file content.
        self.custom_module_bazel = None

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

    def _process_tarball(self) -> Tuple[str, str]:
        """
        Generates the integrity hash and strip prefix for the current Bazel module.
        """
        tarball = self._download_package_tarball()
        integrity =  calculate_integrity_hash_for_file(tarball)
        strip_prefix = f"rosdistro-{self.release_distro}-{self.release_date}"
        try:
            with tarfile.open(tarball, "r:*") as tar:
                for member in tar:
                    if member.name.endswith("package.xml"):
                        f = tar.extractfile(member)
                        if f is not None:
                            content = f.read().decode("utf-8", errors="ignore")
                            if re.search(r"<name>\s*" + re.escape(self.module_name) + r"\s*</name>", content):
                                strip_prefix = Path(member.name).parent.as_posix()
                                if strip_prefix == ".":
                                    strip_prefix = ""
                                break
        except Exception as e:
            print(f"Warning: failed to read tarball {tarball} for {self.module_name}: {e}")
        return integrity, strip_prefix

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
            overlay[rel_path] = calculate_integrity_hash_for_file(overlay_path)
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
            patches[rel_path] = calculate_integrity_hash_for_file(patch_path)
        return patches

    def _write_source_json(self, integrity, strip_prefix):
        """
        Generates the source.json contents for the current Bazel module.
        """
        source_json = {
            "integrity": integrity,
            "url": self.module_url,
            "strip_prefix": strip_prefix,
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

        # First try and add it to the versions. Returns successful if the version is
        # added or already exists. Otherwise, we need to create the file.
        if add_version_to_metadata_json(self.metadata_json_path, self.package_version):
            return

        # Derive from the homepage URL from the GBP repo.
        homepage_url = self.module_url
        parsed = urllib.parse.urlparse(homepage_url)
        if parsed.netloc == "github.com":
            path_parts = parsed.path.strip('/').split('/')
            if len(path_parts) >= 2:
                new_path = '/' + '/'.join(path_parts[:2])
                homepage_url = urllib.parse.urlunparse((parsed.scheme, parsed.netloc, new_path, '', '', ''))

        # Construct the metadata
        metadata = {
            "homepage": homepage_url,
            "maintainers": [
                {
                    "email": "simmers@google.com",
                    "github": "asymingt",
                    "github_user_id": 37671,
                    "name": "Andrew Symington"
                }
            ],
            "versions": [
                self.package_version
            ],
            "yanked_versions": {}
        }
        with open(self.metadata_json_path, 'w') as f:
            json.dump(metadata, f, indent=4)
            f.write('\n')

    def _write_module_dot_bazel(self):
        """
        Generates the MODULE.bazel contents for the current Bazel module.
        """
        ret = get_copyright_header()
        ret += "# ROS package information\n"
        ret += 'module(\n'
        ret += '    name = "{0}",\n'.format(self.module_name)
        ret += '    version = "{0}",\n'.format(self.module_version)
        ret += '    bazel_compatibility = [">=7.2.1"],\n'
        ret += ')\n'
        if self.bcr_deps:
            ret += '\n# BCR dependencies\n'
            for dep_name in sorted(self.bcr_deps.keys()):
                ret += 'bazel_dep(name = "{0}", version = "{1}")\n'.format(
                    dep_name, self.bcr_deps[dep_name])
        if self.custom_module_bazel:
            ret += self.custom_module_bazel
        elif self.rcr_deps:
            ret += '\n# RCR Dependencies\n'
            for dep_name in sorted(self.rcr_deps.keys()):
                ret += 'bazel_dep(name = "{0}", version = "{1}")\n'.format(
                    dep_name, self.rcr_deps[dep_name])
            if self.module_name != 'rosdistro':
                ret += """
bazel_dep(name = "rosdistro", version = "{0}.{1}")

pip_ros = use_extension("@rosdistro//python:defs.bzl", "pip_ros")
use_repo(pip_ros, "pip_ros")
""".format(self.release_distro, self.release_date)
        self.module_file_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.module_file_path, 'w') as f:
            f.write(ret)

    def add_dependency(self, name: str, version: str):
        """
        Add a dependency to the current Bazel module. The input name is
        a rosdep key, and the logic in this function decides whether it
        should be mapped to a BCR module, a RCR module, or be omitted
        entirely. It also resolves the correction version for the dep.
        """
        self.rcr_deps[name] = version

    def generate_module(self):
        """
        Generate the module.
        """
        integrity, strip_prefix = self._process_tarball()
        self._write_module_dot_bazel()
        self._write_source_json(integrity, strip_prefix)
        self._write_metadata_json()
