#!/usr/bin/env python3
# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Scrapes Ubuntu snapshot archives for python3-* packages and outputs
pip-style requirements (package==version) to a file.

Usage:
    bazel run //tools:ubuntu_python_packages -- \
        --ubuntu-version 24.04 \
        --date 2026-01-21 \
        --output $PWD/requirements.in
"""

import argparse
import gzip
import io
import re
import sys
import urllib.request
from typing import Dict

# Maps Ubuntu release version numbers to their codenames.
UBUNTU_CODENAMES = {
    "20.04": "focal",
    "22.04": "jammy",
    "24.04": "noble",
    "24.10": "oracular",
    "25.04": "plucky",
}

COMPONENTS = ["main", "restricted", "universe", "multiverse"]
POCKETS = ["", "-updates"]

def version_to_codename(version: str) -> str:
    """Convert an Ubuntu version string to its release codename."""
    if version not in UBUNTU_CODENAMES:
        supported = ", ".join(sorted(UBUNTU_CODENAMES.keys()))
        raise ValueError(
            f"Unknown Ubuntu version '{version}'. "
            f"Supported versions: {supported}"
        )
    return UBUNTU_CODENAMES[version]


def date_to_snapshot_id(date_str: str) -> str:
    """Convert a date string (YYYY-MM-DD) to a snapshot ID (YYYYMMDDTHHMMSSZ)."""
    match = re.fullmatch(r"(\d{4})-(\d{2})-(\d{2})", date_str)
    if not match:
        raise ValueError(
            f"Invalid date format '{date_str}'. Expected YYYY-MM-DD."
        )
    return f"{match.group(1)}{match.group(2)}{match.group(3)}T000000Z"


def strip_debian_version(version: str) -> str:
    """Strip Debian-specific portions from a version string.

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
    return version


def fetch_packages_gz(snapshot_id: str, codename: str, architecture: str,
                      pocket: str, component: str) -> bytes:
    """Download and return the raw contents of a Packages.gz file."""
    suite = f"{codename}{pocket}"
    url = (
        f"https://snapshot.ubuntu.com/ubuntu/{snapshot_id}"
        f"/dists/{suite}/{component}/binary-{architecture}/Packages.gz"
    )
    print(f"  Fetching {url} ...")
    try:
        with urllib.request.urlopen(url, timeout=60) as resp:
            return resp.read()
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            print(f"    -> 404 Not Found (skipping)")
            return b""
        raise


def parse_packages(data: bytes) -> Dict[str, str]:
    """Parse a Packages.gz blob and return {pip_name: upstream_version}
    for all python3-* packages found."""
    packages: Dict[str, str] = {}
    if not data:
        return packages

    text = gzip.decompress(data).decode("utf-8", errors="replace")

    current_package = None
    current_version = None

    for line in text.splitlines():
        if line.startswith("Package: "):
            current_package = line[len("Package: "):]
            current_version = None
        elif line.startswith("Version: "):
            current_version = line[len("Version: "):]
        elif line == "":
            # End of stanza — process if it's a python3-* package.
            if (current_package and current_version
                    and current_package.startswith("python3-")):
                pip_name = current_package[len("python3-"):]
                upstream = strip_debian_version(current_version)
                # Keep the entry with the higher version if duplicated.
                if pip_name not in packages or packages[pip_name] < upstream:
                    packages[pip_name] = upstream
            current_package = None
            current_version = None

    # Handle last stanza if file doesn't end with a blank line.
    if (current_package and current_version
            and current_package.startswith("python3-")):
        pip_name = current_package[len("python3-"):]
        upstream = strip_debian_version(current_version)
        if pip_name not in packages or packages[pip_name] < upstream:
            packages[pip_name] = upstream

    return packages


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Scrape Ubuntu snapshot archives for python3-* packages and "
            "output pip-style requirements."
        )
    )
    parser.add_argument(
        "--ubuntu-version",
        required=True,
        help="Ubuntu version, e.g. 24.04",
    )
    parser.add_argument(
        "--architecture",
        required=True,
        help="Snapshot architecture, e.g. amd64",
    )
    parser.add_argument(
        "--date",
        required=True,
        help="Snapshot date in YYYY-MM-DD format, e.g. 2026-01-21",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Path to the output requirements.in file",
    )
    args = parser.parse_args()

    codename = version_to_codename(args.ubuntu_version)
    architecture = args.architecture
    snapshot_id = date_to_snapshot_id(args.date)

    print(f"Ubuntu {args.ubuntu_version} ({codename}), "
          f"{architecture} architecture, "
          f"snapshot {snapshot_id}")

    all_packages: Dict[str, str] = {}

    for pocket in POCKETS:
        for component in COMPONENTS:
            data = fetch_packages_gz(snapshot_id, codename, architecture, pocket, component)
            found = parse_packages(data)
            # Merge, keeping higher versions.
            for name, ver in found.items():
                if name not in all_packages or all_packages[name] < ver:
                    all_packages[name] = ver

    print(f"\nFound {len(all_packages)} python3-* packages.")

    # Write sorted output.
    with open(args.output, "w") as f:
        for name in sorted(all_packages.keys()):
            f.write(f"{name}=={all_packages[name]}\n")

    print(f"Wrote requirements to {args.output}")


if __name__ == "__main__":
    main()
