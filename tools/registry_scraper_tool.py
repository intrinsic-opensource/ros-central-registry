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

"""
This tool parses a `registry_scraper_data.yaml` file to determine all of
the repos and packages in this project. Using this, it calculates a diff
between the upstream package version and the changes needed to get a
Bazel build working, and transforms this to a Bazel module.
"""

import argparse
import hashlib
import base64
import json
import shutil
import yaml
import urllib3
from git import Repo
from pathlib import Path
from python.runfiles import runfiles
                                        

def calculate_bazel_integrity(uri):
    """Given a local or remote file URI, download and calculate integrity"""
    file_path = uri
    # Special case, this is probably a URL.
    if not Path(uri).exists():
        uri_split = uri.split("/")
        file_path = Path("/tmp/downloads/" + "-".join([uri_split[-5], uri_split[-1]]))
        # If we haven't yet downloaded this file, let's go ahead and do it.
        if not file_path.exists():
            file_path.parent.mkdir(parents=True, exist_ok=True)
            http = urllib3.PoolManager()
            r = http.request('GET', uri, preload_content=False)
            with open(file_path, 'wb') as out:
                while True:
                    data = r.read(212992) # cat /proc/sys/net/core/rmem_default
                    if not data:
                        break
                    out.write(data)
            r.release_conn()
    # Calculate the hash for the file.
    sha256_hash = hashlib.sha256()
    block_size = 65536  # 64 KB
    try:
        with open(file_path, "rb") as f:
            for chunk in iter(lambda: f.read(block_size), b""):
                sha256_hash.update(chunk)
    except IOError as e:
        print(f"Error reading file: {e}")
        return None
    return f"sha256-" + str(base64.b64encode(sha256_hash.digest()).decode('utf-8'))

def update_metadata_json(metadata_json, ghsuffix, version):
    """Update a metadata.json file for a given module"""
    metadata_json.parent.mkdir(parents=True, exist_ok=True)
    if metadata_json.exists():
        with open(metadata_json, 'r') as f:
            module_data = json.load(f)
            module_versions = set(module_data["versions"])
            module_versions.add(version)
            module_data["versions"] = sorted(list(module_versions))
    else:
        module_data = {
            "homepage": f"https://github.com/{ghsuffix}",
            "maintainers": [
                {
                    "email": "intrinsic-opensource@users.noreply.github.com",
                    "name": "intrinsic-opensource"
                }
            ],
            "repository": [
                f"github:{ghsuffix}"
            ],
            "versions": [
                version
            ],
            "yanked_versions": {}
        }
    with open(metadata_json, 'w') as f:
        json.dump(module_data, f, indent=4)

def update_source_json(source_json, ghsuffix, upstream, overlay = {}, patches = {}, strip = ""):
    """Update a metadata.json file for a given module"""
    source_json.parent.mkdir(parents=True, exist_ok=True)
    url = f"https://github.com/{ghsuffix}/archive/refs/tags/{upstream}.tar.gz"
    strip_prefix = ghsuffix.split("/")[-1] + "-" + upstream.removeprefix("v").replace("/", "-")
    if strip != "":
        strip_prefix += "/" + strip
    source_data = {
        "url": url,
        "integrity": calculate_bazel_integrity(url),
        "strip_prefix": strip_prefix,
    }
    if overlay:
        source_data["overlay"] = overlay
    if patches:
        source_data["patches"] = patches
        source_data["patch_strip"] = len(strip_prefix.split("/"))
    with open(source_json, 'w') as f:
        json.dump(source_data, f, indent=4)

def update_overlay_file(overlay_dest, overlay_src):
    """Update a metadata.json file for a given module"""
    overlay_dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(overlay_src, overlay_dest)

def update_patch_file(patch_dest, patch_content):
    """Update a patch file with content"""
    patch_dest.parent.mkdir(parents=True, exist_ok=True)
    patch_content = patch_content.replace("\n\\ No newline at end of file", "")
    with open(patch_dest, "w") as file:
        file.write(patch_content)

def repos_from_submodules(repo_base_path, repo_yaml_file, package=None):
    """Process all repos in the yaml file"""
    base_out_path = repo_base_path / 'docs' / 'modules'

    # Try and open the file with all the repos
    try:
        with repo_yaml_file.open('r') as f:
            yaml_data = yaml.safe_load(f)
        print("YAML data loaded successfully:")
    except FileNotFoundError:
        print(f"Error: The YAML file '{repo_yaml_file}' was not found.")
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")

    for repo_name, repo_data in yaml_data.items():
        print("Processing", repo_name)
        repo_folder = repo_data.get('folder', 'submodules')
        repo_version = repo_data["version"]
        repo_ghsuffix = repo_data["github"]
        repo_upstream = repo_data.get('upstream', repo_version)
        repo_path = repo_base_path / repo_folder / repo_name

        # Collect diffs
        repo = Repo(repo_path)
        commit_base = repo.commit(repo_upstream)
        commit_feat = repo.commit("HEAD")
        diff_index = commit_base.diff(commit_feat)
        repo_overlay = {}
        repo_patches = {}
        for diff in diff_index:
            if diff.change_type == 'M':  # 'M' for modified
                repo_patches[diff.b_path] = repo.git.diff(commit_base, commit_feat, diff.b_path)
            else:
                overlay_path = str(repo_path / diff.b_path)
                repo_overlay[diff.b_path] = calculate_bazel_integrity(overlay_path)
        
        # Extract modules for this repo.
        repo_modules = {}
        if 'modules' in repo_data.keys():
            for module_name, module_path in repo_data['modules'].items():
                repo_modules[module_name] = module_path
        else:
            repo_modules[repo_name] = ""

        # Process modules
        for module_name, module_subfolder in repo_modules.items():
            if package is not None and package != module_name:
                continue
            print("+", module_name)

            # Clear anything that's already there.
            shutil.rmtree(base_out_path / module_name, ignore_errors=True)

            # Get the path to the module folder of the repo
            module_path = repo_base_path / repo_folder / repo_name / module_subfolder

            # Update the metadata.json file.
            metadata_json = base_out_path / module_name / 'metadata.json'
            update_metadata_json(metadata_json, repo_ghsuffix, repo_version)

            # Create overlay files.
            overlay = {}
            for overlay_rel_path, overlay_integrity in repo_overlay.items():
                overlay_src_path =  repo_path / overlay_rel_path
                if not str(overlay_src_path).startswith(str(module_path) + "/"):
                    continue
                strip_prefix = str(overlay_src_path).removeprefix(str(module_path) + "/")
                overlay_dst_path = base_out_path / module_name / repo_version / 'overlay' / strip_prefix
                update_overlay_file(overlay_dst_path, overlay_src_path)
                overlay[strip_prefix] = overlay_integrity

            # Create patch files
            patches = {}
            for patch_rel_path, patch_content in repo_patches.items():
                patch_src_path =  repo_path / patch_rel_path
                if not str(patch_src_path).startswith(str(module_path) + "/"):
                    continue
                strip_prefix = str(patch_src_path).removeprefix(str(module_path) + "/")
                patch_name = "fix_" + str(strip_prefix).replace("/", "_").replace(".", "_") + ".patch"
                patch_dst_path = base_out_path / module_name / repo_version / 'patches' / patch_name
                update_patch_file(patch_dst_path, patch_content)
                patches[patch_name] = calculate_bazel_integrity(patch_dst_path)

            # Copy over the MODULE.bazel file.
            src_module_bazel = base_out_path / module_name / repo_version / 'overlay' / 'MODULE.bazel'
            dst_module_bazel = base_out_path / module_name / repo_version / 'MODULE.bazel'
            dst_module_bazel.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(src_module_bazel, dst_module_bazel)

            # Patches are done at the repo level, so we need to strip out the subfolder.
            strip = module_subfolder

            # Update the source.json file.
            source_json = base_out_path / module_name / repo_version / 'source.json'
            update_source_json(source_json, repo_ghsuffix, repo_upstream, overlay, patches, strip)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("workspace", help="Path to the bazel workspace")
    parser.add_argument("-p", "--package", type=str, help="Target package name (optional)")
    args = parser.parse_args()
    repo_base_path = Path(args.workspace)
    repo_yaml_file = Path(runfiles.Create().Rlocation('_main/tools/registry_scraper_data.yaml'))
    repos_from_submodules(repo_base_path, repo_yaml_file, args.package)
