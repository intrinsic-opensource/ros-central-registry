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

import base64
import hashlib
import json
import re
from pathlib import Path
from typing import Dict
from bazelflore.utils.copyright import get_copyright_header

def calculate_integrity_hash_for_file(file_path: Path) -> str:
    """
    Calculate a bazel integrity hash for file.
    """
    sha256_hash = hashlib.sha256()
    with open(file_path, "rb") as f:
        for byte_block in iter(lambda: f.read(4096), b""):
            sha256_hash.update(byte_block)
    return "sha256-" + base64.b64encode(sha256_hash.digest()).decode()
    
def add_version_to_metadata_json(metadata_json_path: Path, package_version: str) -> bool:
    """
    Update the metadata.json file for the current Bazel module version, if needed.
    Returns True if the file was updated, False otherwise.
    """
    if not metadata_json_path.exists():
        return False
    with open(metadata_json_path, 'r') as f:
        try:
            metadata = json.load(f)
        except:
            return False
    if package_version not in metadata["versions"]:
        metadata["versions"].append(package_version)
        metadata["versions"].sort()
        with open(metadata_json_path, 'w') as f:
            json.dump(metadata, f, indent=4)
            f.write('\n')
    return True

def regenerate_integrity_hashes(module_dir: Path) -> bool:
    """
    Regenerate the integrity hashes for the module directory.
    Returns True if the file was updated, False otherwise.
    """
    if not module_dir.exists():
        return False

    # Open the existing source.json file.
    source_json_path = module_dir / "source.json"
    with open(source_json_path, 'r') as f:
        try:
            source = json.load(f)
        except:
            return False
    
    # Delete any overlay and patches that might already exist.
    try:
        del source["overlay"]
    except KeyError:
        pass
    try:
        del source["patches"]
    except KeyError:
        pass

    # Perform the integrity check for the overlay and patches.
    for name in ["overlay", "patches"]:
        targ_dir = module_dir / name
        if targ_dir.exists():
            source[name] = {}
            for targ_file in targ_dir.iterdir():
                if targ_file.is_file():
                    rel_path = targ_file.relative_to(targ_dir)
                    source[name][str(rel_path)] = calculate_integrity_hash_for_file(targ_file)

    # Overwrite the source.json file.
    with open(source_json_path, 'w') as f:
        json.dump(source, f, indent=4)
        f.write('\n')
    return True

def add_boilerplate_build_file(overlay_dir: Path) -> bool:
    """
    Add a boilerplate BUILD.bazel file to the overlay directory.
    """
    if not overlay_dir.exists():
        return False
    with open(overlay_dir / "BUILD.bazel", 'w') as f:
        f.write(get_copyright_header())
    return True

def increment_version(version: str) -> str:
    """
    Increment the version of a package.
    """
    version_parts = version.split(".")
    if len(version_parts) < 2:
        RuntimeError(f"Malformed version: {version}")
    
    # If this has already been patched, increment the patch version.
    if version_parts[-2] == "rcr":
        curr_patch_version = int(version_parts[-1])
        next_patch_version = str(curr_patch_version + 1)
        return ".".join(version_parts[:-1] + [next_patch_version])

    # Otherwise, this is the first patch release.
    return "{0}.rcr.0".format(version)

def scan_module_for_dependencies(module_dir: Path) -> Dict[str, str]:
    """
    Returns a list of package names and their versions.
    """
    packages = {}
    with open(module_dir / "MODULE.bazel", 'r') as f:
        content = f.read()
        matches = re.findall(r'bazel_dep\(\s*name\s*=\s*"([^"]+)"\s*,\s*version\s*=\s*"([^"]+)"', content)
        for name, version in matches:
            packages[name] = version
    return packages