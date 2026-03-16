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
Scrapes Ubuntu snapshot archives for package metadata
"""

import argparse
import gzip
import io
import re
import sys
import urllib.request
from collections import namedtuple
from pathlib import Path
from typing import Dict

COMPONENTS = ["main", "restricted", "universe", "multiverse"]

POCKETS = ["", "-updates"]

DebSource = namedtuple('DebSource', ['version', 'source'])

class DebWorker:
    def __init__(self, working_directory: Path, distro: str, architecture: str, date: str):
        """
        Initialize the Ubuntu snapshot.
        
        Args:
            working_directory: The working directory.
            distro: The distribution (e.g., "jammy").
            architecture: The architecture (e.g., "amd64").
            date: The date (e.g., "2026-03-16").
        """

        match = re.fullmatch(r"(\d{4})-(\d{2})-(\d{2})", date)
        if not match:
            raise ValueError(
                f"Invalid date format '{date}'. Expected YYYY-MM-DD."
            )

        self.working_directory = working_directory
        self.distro = distro
        self.architecture = architecture
        self.snapshot_id = f"{match.group(1)}{match.group(2)}{match.group(3)}T000000Z"


    def _fetch_packages_gz(self, working_directory: Path, snapshot_id: str, distro: str, architecture: str,
                        pocket: str, component: str) -> bytes:
        """
        Download and return the raw contents of a Packages.gz file.
        
        Args:
            working_directory: The working directory.
            snapshot_id: The snapshot ID (e.g., "20260316T000000Z").
            distro: The distribution (e.g., "jammy").
            architecture: The architecture (e.g., "amd64").
            pocket: The pocket (e.g., "").
            component: The component (e.g., "main").
        
        Returns:
            The raw contents of the Packages.gz file.
        """
        suite = f"{distro}{pocket}"
        cache_dir = working_directory / ".cache"
        cache_file = cache_dir / f"{snapshot_id}_{suite}_{component}_{architecture}.gz"

        if cache_file.exists():
            return cache_file.read_bytes()

        url = (
            f"https://snapshot.ubuntu.com/ubuntu/{snapshot_id}"
            f"/dists/{suite}/{component}/binary-{architecture}/Packages.gz"
        )
        try:
            with urllib.request.urlopen(url, timeout=60) as resp:
                data = resp.read()
                if data:
                    cache_dir.mkdir(parents=True, exist_ok=True)
                    cache_file.write_bytes(data)
                return data
        except urllib.error.HTTPError as exc:
            if exc.code == 404:
                return b""
            raise

    def _parse_packages(self, data: bytes) -> Dict[str, DebSource]:
        """
        Parse the raw contents of a Packages.gz file into a dictionary.
        
        Args:
            data: The raw contents of a Packages.gz file.
        
        Returns:
            A dictionary of debians, where the keys are the package names and the values are dictionaries containing the version and source of the package.
        """
        debs: Dict[str, DebSource] = {}
        if not data:
            return debs

        text = gzip.decompress(data).decode("utf-8", errors="replace")

        current_package = None
        current_version = None
        current_source = None

        def maybe_add_package(package: str, version: str, source: str):
            """
            Add a package to the dictionary if it is not already present or if the new version is greater than the existing version.
            
            Args:
                package: The name of the package.
                version: The version of the package.
                source: The source of the package.
            """
            if not (package and version and source):
                return
            version = self._strip_debian_version(version.strip())
            replace = True
            if package in debs.keys():
                if debs[package].version > version:
                    replace = False
            if replace:
                debs[package] = DebSource(version, source)

        for line in text.splitlines():
            if line.startswith("Package: "):
                current_package = line[len("Package: "):]
                current_version = None
                current_source = None
            elif line.startswith("Version: "):
                current_version = line[len("Version: "):]
            elif line.startswith("Source: "):
                current_source = line[len("Source: "):]
            elif line == "":
                maybe_add_package(current_package, current_version, current_source)
                current_package = None
                current_version = None
                current_source = None

        # Handle last stanza if file doesn't end with a blank line.
        maybe_add_package(current_package, current_version, current_source)

        return debs

    def _strip_debian_version(self, version: str) -> str:
        """
        Strip Debian-specific portions from a version string.

        Debian versions follow the format [epoch:]upstream[-debian_revision].
        The upstream portion may also carry repackaging suffixes such as
        +dfsg, +ds, +git..., +really..., etc.  We strip all of these to
        arrive at a clean pip-compatible version.

        Examples:
            "1.2.3-1ubuntu2"            -> "1.2.3"
            "2:1.0.0-1build1"           -> "1.0.0"
            "1.26.4+ds-2ubuntu1"        -> "1.26.4"
            "2.9.2+dfsg1-1"             -> "2.9.2"
            "0.5.0+git20201231.344346a" -> "0.5.0"
            "3.4.5"                     -> "3.4.5"
        """
        # Strip epoch (everything up to and including the first colon).
        if ":" in version:
            version = version.split(":", 1)[1]
        # Strip Debian revision (everything from the last hyphen onward).
        if "-" in version:
            version = version.rsplit("-", 1)[0]
        # Strip Debian repackaging suffixes (+dfsg, +ds, +git..., +really, etc.).
        version = re.split(r"\+(?:dfsg|ds|git|really|repack|nmu|b\d)", version)[0]
        # Strip any remaining +<suffix> that isn't part of a normal version.
        version = re.split(r"\+[a-zA-Z]", version)[0]
        # Strip Debian pre-release tilde suffixes (e.g. 1.2.3~rc1 -> 1.2.3).
        version = version.split("~")[0]
        # Strip any trailing plus or minus signs.
        version = version.rstrip("+-")
        return version

    def fetch_packages(self, component: str) -> Dict[str, DebSource]:
        """
        Fetch and parse all packages from the snapshot into a {name : DebSource} dictionary.
        """
        all_debs: Dict[str, DebSource] = {}
        for pocket in POCKETS:
            data = self._fetch_packages_gz(
                self.working_directory, 
                self.snapshot_id, 
                self.distro, 
                self.architecture, 
                pocket, 
                component
            )
            debs = self._parse_packages(data)    
            for name, info in debs.items():
                if name in all_debs and all_debs[name].version > info.version:
                    continue
                all_debs[name] = info
        return all_debs
