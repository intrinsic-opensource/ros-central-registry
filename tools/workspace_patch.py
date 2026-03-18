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
Generate a RCR patch from a given release workspace
"""

import argparse
import datetime
import shutil
from pathlib import Path
from termcolor import COLORS, HIGHLIGHTS
from typing import Optional, Dict, List
from yaspin import yaspin
from yaspin.spinners import Spinners
from bazelflore.utils.bzlmod import increment_version
from bazelflore.utils.bzlmod import find_latest_patch
from bazelflore.utils.bzlmod import find_release
from bazelflore.utils.bzlmod import scan_module_for_dependencies
from bazelflore.utils.bzlmod import get_module_diff
from bazelflore.utils.bzlmod import load_patches_and_overlays
from bazelflore.utils.bzlmod import regenerate_integrity_hashes
from bazelflore.utils.bzlmod import add_version_to_metadata_json

def main():

    # Gather arguments
    parser = argparse.ArgumentParser(description="Generate a RCR patch from a given release workspace")
    parser.add_argument('working_directory', type=Path)
    parser.add_argument(
        "--workspace",
        required=True,
        type=str,
        help="Bazel 'ros' module release version, e.g. rolling.2026-01-21",
    )
    parser.add_argument(
        "--packages",
        nargs='+',
        type=str,
        help="List of packages to include in the workspace",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite the workspace directory if it exists",
    )
    args = parser.parse_args()

    with yaspin(text="Generating patch set from workspace", color="cyan") as sp:
        # REF: rolling.2026-01-21.        (bare release without patches)
        # OLD: rolling.2026-01-21,bcr.2   (what to start with)
        # NEW: rolling.2026-01-21.bcr.3   (what we are adding)
        modules_dir = args.working_directory / "modules"

        # Find the release from the given workspace
        ref_patch = find_release(args.workspace)
        ref_dir = args.working_directory / "workspace" / ref_patch
        sp.write("> Found REF: {0}".format(ref_patch))

        # Given the release, we need to find the OLD and NEW patch versions.
        old_patch = find_latest_patch(args.working_directory, ref_patch)
        sp.write("> Found OLD: {0}".format(old_patch))

        # Make sure the workspace exists
        new_dir = args.working_directory / "workspace" / args.workspace
        if not new_dir.exists():
            raise RuntimeError("Workspace {0} does not exist".format(args.workspace))
        sp.write("> Found NEW: {0}".format(args.workspace))

        # Get all the old packages
        old_module_file = args.working_directory / "modules" / "ros" / old_patch / 'MODULE.bazel'
        old_packages = scan_module_for_dependencies(old_module_file, modules_dir)
        old_packages["ros"] = old_patch
        sp.write("> Found {0} OLD packages".format(len(old_packages)))

        # Find all the packages in the REF release
        # sp.write("> Scanning REF release for packages")
        # ref_packages = scan_module_for_dependencies(ref_dir / 'MODULE.bazel', modules_dir)
        # ref_packages["ros"] = release
        # sp.write("> Found {0} REF packages".format(len(ref_packages)))

        # Find the new patch version.
        packages = {
            d.name.removesuffix("+") : d.name for d in (new_dir / "vendor").iterdir()
            if d.is_dir() and d.name.endswith("+")
        }
        sp.write("> Found {0} NEW packages".format(len(packages)))
        if args.packages is not None:
            sp.write("> Including packages: {0}".format(", ".join(args.packages)))
            packages = {key: value for key, value in packages.items() if key in args.packages}
            sp.write("> Filtered to {0} NEW package".format(len(packages)))

        # Calculating diffs for all packages
        updated_modules = {}
        sp.write("> Calculating which modules need updating")
        for package_name, package_dir in sorted(packages.items()):
            #sp.write("> Processing {0}".format(package_name))

            # Find the patches and overlays for the package, by comparing the NEW and REF workspaces.
            ref_package_dir = ref_dir / "vendor" / package_dir
            new_package_dir = new_dir / "vendor" / package_dir
            new_patches, new_overlays = get_module_diff(ref_package_dir, new_package_dir)            
            #sp.write("  + Found {0} patches and {1} overlays in NEW".format(len(new_patches), len(new_overlays)))

            # Load the patches and overlays from the OLD module directory.
            old_package_version = old_packages[package_name]
            old_patches, old_overlays = load_patches_and_overlays(
                args.working_directory / "modules" / package_name / old_package_version)
            #sp.write("  + Found {0} patches and {1} overlays in OLD".format(len(old_patches), len(old_overlays)))

            # If there are no changes, leave the module as-is.
            if (old_patches == new_patches) and (old_overlays == new_overlays):
                continue
            
            # Increment the version.
            new_package_version = increment_version(old_package_version)
            sp.write("  + Incrementing {0} version from {1} to {2}".format(
                package_name, old_package_version, new_package_version))            
            
            # Copy the old module
            old_module_dir = args.working_directory / "modules" / package_name / old_package_version
            new_module_dir = args.working_directory / "modules" / package_name / new_package_version
            shutil.copytree(old_module_dir, new_module_dir, dirs_exist_ok=True)

            # Replace the overlay and patches files.
            shutil.rmtree(new_module_dir / "patches", ignore_errors=True)
            shutil.rmtree(new_module_dir / "overlay", ignore_errors=True)
            for patch_name, patch_content in new_patches.items():
                patch_path = new_module_dir / "patches" / patch_name
                patch_path.parent.mkdir(parents=True, exist_ok=True)
                with open(patch_path, 'w') as f:
                    f.write(patch_content)  
            for overlay_name, overlay_content in new_overlays.items():
                overlay_path = new_module_dir / "overlay" / overlay_name
                overlay_path.parent.mkdir(parents=True, exist_ok=True)
                with open(overlay_path, 'w') as f:
                    f.write(overlay_content)  

            # Update the source.json file.
            regenerate_integrity_hashes(new_module_dir)

            # Update the metadata.json file
            metadata_json_path = args.working_directory / "modules" / package_name / "metadata.json"
            add_version_to_metadata_json(metadata_json_path, new_package_version)

            # List this as an updated module.
            updated_modules[package_name] = new_package_version
        sp.write("> Need to update {0} modules with new content".format(len(updated_modules)))

        # This is where is gets a little complicated. We need to do a transitive update of bazel_dep
        # calls in all RCR modules to point to the new versions. Since we can't edit an existing
        # module, we have to create a new version, even if all that has changed is the call in the
        # MODULE.bazel file. Annoyingly, if foo calls bar which calls baz, and baz is updated, then
        # foo and bar will need to be updated as well. We have to do this recursively until we have
        # updated all modules that are called by the updated modules.
        sp.write("> Calculating transitive updates to bazel_dep")
        updated_at_least_one_module_version = True
        while updated_at_least_one_module_version:
            updated_at_least_one_module_version = False
            for package_name, old_package_version in old_packages.items():
                # If we've already updated this module, skip it.
                if package_name in updated_modules.keys():
                    continue
                old_module_file = args.working_directory / "modules" / package_name / old_package_version / "MODULE.bazel"
                old_dependencies = scan_module_for_dependencies(old_module_file, modules_dir)
                needs_update = False
                for dependency in old_dependencies:
                    if dependency in updated_modules.keys():
                        needs_update = True
                if needs_update:
                    new_package_version = increment_version(old_package_version)
                    sp.write("  + Incrementing {0} from {1} to {2}".format(
                        package_name, old_package_version, new_package_version))

                    # Copy the old module
                    old_module_dir = args.working_directory / "modules" / package_name / old_package_version
                    new_module_dir = args.working_directory / "modules" / package_name / new_package_version
                    shutil.copytree(old_module_dir, new_module_dir, dirs_exist_ok=True)

                    # Update the metadata.json file
                    metadata_json_path = args.working_directory / "modules" / package_name / "metadata.json"
                    add_version_to_metadata_json(metadata_json_path, new_package_version)

                    # List this as an updated module.
                    updated_modules[package_name] = new_package_version
                    updated_at_least_one_module_version = True

        sp.write("> Need to update {0} modules with new dependencies".format(len(updated_modules)))

        # TODO(asymingt
        # Now we need to update all bazel_dep calls in all RCR modules to point to the new versions.
        # sp.write("> Updating bazel_dep calls in all RCR modules to point to new modules")
        # for package_name, new_package_version in updated_modules.items():
        # update_bazel_dep_calls(old_module_file, new_module_file)
        sp.ok("✔")


if __name__ == "__main__":
    main()