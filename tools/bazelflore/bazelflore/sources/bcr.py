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
Scrapes BCR snapshot for package version information.
"""

import os
import requests
import json
import shutil
import tarfile
import fnmatch
from collections import namedtuple
from pathlib import Path
from typing import Dict, List

BcrSource = namedtuple('BcrSource', ['versions'])

class BcrWorker:
    """
    Class for fetching package information from the BCR.
    """
 
    def __init__(self, working_directory: Path):
        """
        Initialize the Registry with the working directory, which is used to cache
        the BCR snapshot to avoid repeated downloads.
        """
        self.working_directory = working_directory

    def _get_latest_commit_sha(self):
        """
        Fetches the SHA of the latest commit on a specified GitHub branch.
        """
        url = f"https://api.github.com/repos/bazelbuild/bazel-central-registry/branches/main"
        try:
            response = requests.get(url)
            response.raise_for_status()  # Raise an exception for bad status codes (4XX or 5XX)
            branch_data = response.json()
            latest_commit_sha = branch_data['commit']['sha']
            return latest_commit_sha
        except requests.exceptions.RequestException as e:
            print(f"Error fetching data from GitHub API: {e}")
            return None
        except json.JSONDecodeError:
            print("Error decoding JSON response")
            return None

    def _download_commit_tarball(self, commit_sha):
        """
        Downloads the tarball of a specific commit from a GitHub repository.
        """
        cache_dir = self.working_directory / ".cache"
        cache_dir.mkdir(parents=True, exist_ok=True)
        cache_file = cache_dir / f"bazel-central-registry-{commit_sha}.tar.gz"
        if cache_file.exists():
            return cache_file
        url = f"https://api.github.com/repos/bazelbuild/bazel-central-registry/tarball/{commit_sha}"
        try:
            with requests.Session() as s:
                headers = {'User-Agent': 'Python-Script-Tarball-Downloader'}
                response = s.get(url, headers=headers, stream=True)
                response.raise_for_status() # Raise an exception for bad status codes
                with open(cache_file, 'wb') as f:
                    shutil.copyfileobj(response.raw, f)
            return cache_file
        except requests.exceptions.RequestException as e:
            return None

    def _parse_tarball(self, tarball_file : Path) -> Dict[str, BcrSource]:
        """
        Parse the tarball and extract package information.
        """
        bcr_sources: Dict[str, BcrSource] = {}
        if not tarball_file or not tarball_file.exists():
            return bcr_sources

        # Open up the archive and scan the modules folder for module names
        # and versions. Aggregate into a dictionary.
        with tarfile.open(tarball_file, "r:gz") as tar:
            for member in tar.getmembers():
                if member.isfile() and member.name.endswith("metadata.json"):
                    parts = member.name.split("/")
                    if len(parts) >= 4 and parts[-3] == "modules":
                        module_name = parts[-2]
                        f = tar.extractfile(member)
                        if f:
                            try:
                                metadata = json.load(f)
                                if "versions" in metadata and metadata["versions"]:
                                    bcr_sources[module_name] = BcrSource(metadata["versions"])
                            except json.JSONDecodeError:
                                print(f"Error decoding JSON from {member.name}")
        return bcr_sources

    def fetch_packages(self) -> Dict[str, BcrSource]:
        """
        Fetch and parse all packages from the snapshot, optionally filtered by component pattern.
        """
        bcr_sources: Dict[str, BcrSource] = {}

        commit_sha = self._get_latest_commit_sha()
        if not commit_sha:
            return bcr_sources

        tarball_file = self._download_commit_tarball(commit_sha) 
        if not tarball_file:
            return all_modules

        return self._parse_tarball(tarball_file)

