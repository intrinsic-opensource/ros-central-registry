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
import datetime
import difflib
import filecmp
import hashlib
import json
import re
from pathlib import Path
from typing import Dict, Optional, Tuple
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

def scan_module_for_dependencies(
    module_dot_bazel: Path,
    modules_path : Path,
    include_bcr : bool = False,
    include_rcr : bool = True) -> Dict[str, str]:
    """
    Returns a list of package names and their versions.
    """
    packages = {}
    with open(module_dot_bazel, 'r') as f:
        content = f.read()
        matches = re.findall(r'bazel_dep\(\s*name\s*=\s*"([^"]+)"\s*,\s*version\s*=\s*"([^"]+)"', content)
        for name, version in matches:
            is_rcr_module = (modules_path / name).exists()
            if (include_rcr and is_rcr_module) or (include_bcr and not is_rcr_module):
                packages[name] = version
    return packages


def find_latest_patch(working_directory: Path, release: str) -> Optional[Path]:
    """
    Find the latest patch for the given release.
    """
    modules_dir = working_directory / "modules" / "ros"

    # Extract the distro and date from the distribution.
    release_parts = release.split(".")
    if len(release_parts) != 2:
        raise RuntimeError("{0} is not in the format <distribution>.<date>".format(release))
    needle_distro, needle_date_str = release_parts
    needle_date = datetime.datetime.strptime(needle_date_str, "%Y-%m-%d")

    # Find all candidate patch releases.
    candidate_patch_releases = []
    for x in modules_dir.iterdir():
        if not x.is_dir():
            continue
        release = x.name
        release_parts = release.split(".")
        if len(release_parts) != 4:
            continue
        haystack_distro, haystack_date, _, haystack_patch = release_parts
        haystack_date = datetime.datetime.strptime(haystack_date, "%Y-%m-%d")
        if haystack_distro == needle_distro and haystack_date == needle_date:
            candidate_patch_releases.append((haystack_date, int(haystack_patch), release))

    # We have no candidates, so there is no previous release. This should only
    # ever happen when we run this for the very first time for a distro.
    if not candidate_patch_releases:
        return None

    # Sort by date descending, then patch version descending
    candidate_patch_releases.sort(key=lambda x: (x[0], x[1]), reverse=True)

    # Return the release directory that was chosen.
    return candidate_patch_releases[0][2]

def find_release(workspace: str) -> str:
    """
    Find the release for the given workspace.
    """
    workspace_parts = workspace.split(".")
    if len(workspace_parts) != 4:
        raise RuntimeError("{0} is not in the format <distribution>.<date>.<rcr>.<patch>".format(workspace))
    return ".".join(workspace_parts[:2])

def patch_file_name_from_rel_path(rel_path: Path) -> str:
    """
    Convert a relative path to a patch file name.
    """
    path_file_name = str(rel_path)
    path_file_name = path_file_name.replace("/", "__")
    path_file_name = path_file_name.replace(".", "__")
    return path_file_name + ".patch"

def get_module_diff(ref_package_dir : Path, new_package_dir : Path) -> Tuple[Dict[str, str], Dict[str, str]]:
    """
    Get the diff between two package directories.
    """
    patches = {}
    overlays = {}

    for x in new_package_dir.rglob('*'):
        if not x.is_file() or x.name == "MODULE.bazel":
            continue
        rel_path = x.relative_to(new_package_dir)
        ref_path = ref_package_dir / rel_path
        new_path = x
        if ref_path.exists():
            if filecmp.cmp(ref_path, new_path):
                continue
            
            with open(ref_path, 'r') as f_ref, open(new_path, 'r') as f_new:
                diff_lines = difflib.unified_diff(
                    f_ref.readlines(),
                    f_new.readlines(),
                    fromfile=f"a/{rel_path}",
                    tofile=f"b/{rel_path}"
                )
                patches[patch_file_name_from_rel_path(rel_path)] = ''.join(diff_lines)
        else:
            with open(x, 'r') as f:
                overlays[str(rel_path)] = f.read()
    for x in ref_package_dir.rglob('*'):
        if not x.is_file() or x.name == "MODULE.bazel":
            continue
        rel_path = x.relative_to(ref_package_dir)
        ref_path = x
        new_path = new_package_dir / rel_path
        if not new_path.exists():
            with open(ref_path, 'r') as f_ref:
                diff_lines = difflib.unified_diff(
                    f_ref.readlines(),
                    [],
                    fromfile=f"a/{rel_path}",
                    tofile=f"/dev/null"
                )
                patches[patch_file_name_from_rel_path(rel_path)] = ''.join(diff_lines)
    return patches, overlays

def load_patches_and_overlays(module_dir: Path) -> Tuple[Dict[str, str], Dict[str, str]]:
    """
    Load the patches and overlays from the source.json file.
    """
    with open(module_dir / "source.json", 'r') as f:
        source = json.load(f)
    augmentations = {"patches": {}, "overlay": {}}
    for augmentation_type in augmentations.keys():
        for path in source.get(augmentation_type, {}).keys():
            with open(module_dir / augmentation_type / path, 'r') as f:
                augmentations[augmentation_type][path] = f.read()
    return augmentations["patches"], augmentations["overlay"]
