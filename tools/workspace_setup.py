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
Setup a new workspace for a RCR patch set.
"""

import argparse
import datetime
from pathlib import Path
from termcolor import COLORS, HIGHLIGHTS
from typing import Optional, Dict
from yaspin import yaspin
from yaspin.spinners import Spinners
from bazelflore.utils.bzlmod import increment_version
from bazelflore.utils.bzlmod import scan_module_for_dependencies
from bazelflore.utils.copyright import get_copyright_header

def _find_latest_patch(working_directory: Path, release: str) -> Optional[Path]:
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

def _setup_patch_workspace(workspace_dir: Path, module_name: str, module_version: str, packages: Dict[str, str]):
    """
    Setup a new workspace for a RCR patch set.
    """

    # Create a .bazelversion file
    dot_bazelversion_file = workspace_dir / ".bazelversion"
    with open(dot_bazelversion_file, 'w') as f:
        f.write("9.0.0")

    # Create a .bazelignore file
    dot_bazelignore_file = workspace_dir / ".bazelignore"
    with open(dot_bazelignore_file, 'w') as f:
        f.write("base\n")
        f.write("feat\n")

    # Create a .bazelrc file
    dot_bazelrc_file = workspace_dir / ".bazelrc"
    with open(dot_bazelrc_file, 'w') as f:
        repo_args = [ f"\t--repo=@{p} \\" for p in sorted(packages.keys())]
        f.write(get_copyright_header())
        f.write("""
common --registry=file://%workspace%/../.. --registry=https://bcr.bazel.build
common --remote_cache=https://storage.googleapis.com/intrinsic-opensource-buildcache
common --remote_upload_local_results=false
common --remote_cache_compression=true
common --test_env=ROS_HOME=".ros"
common --incompatible_default_to_explicit_init_py
common --incompatible_strict_action_env
test --sandbox_default_allow_network=true
build --cxxopt="-std=c++17"
build --action_env="BAZEL_DO_NOT_DETECT_CPP_TOOLCHAIN=1"
build --@protobuf//bazel/toolchains:prefer_prebuilt_protoc
common:distribution --target_pattern_file=distribution.txt
vendor --vendor_dir=vendor {0}
""".format("\n".join(repo_args)))

    # Create a VENDOR.bazel file
    vendor_dot_bazel = workspace_dir / "vendor" / "VENDOR.bazel"
    vendor_dot_bazel.parent.mkdir(parents=True, exist_ok=True)
    with open(vendor_dot_bazel, 'w') as f:
        for package in sorted(packages.keys()):
            f.write("pin(\"@@{0}\")\n".format(package))

    # Create a distribution.txt file
    distribution_txt_file = workspace_dir / "distribution.txt"
    with open(distribution_txt_file, 'w') as f:
        for package in sorted(packages.keys()):
            f.write("@{0}//...\n".format(package))

    # Create a MODULE.bazel file
    module_dot_bazel_files = workspace_dir / "MODULE.bazel"
    with open(module_dot_bazel_files, 'w') as f:
        f.write(get_copyright_header())
        f.write(f"""module(
    name = "{module_name}",
    version = "{module_version}",
)

# BCR deps
bazel_dep(name = "platforms", version = "1.0.0")
bazel_dep(name = "protobuf", version = "34.0.bcr.1")
bazel_dep(name = "toolchains_llvm", version = "1.6.0")

llvm = use_extension("@toolchains_llvm//toolchain/extensions:llvm.bzl", "llvm")
llvm.toolchain(llvm_version = "20.1.7")
use_repo(llvm, "llvm_toolchain")

register_toolchains("@llvm_toolchain//:all")

# RCR deps
""")
        for package in sorted(packages.keys()):
            f.write("bazel_dep(name = \"{0}\", version = \"{1}\")\n".format(package, packages[package]))

def main():

    # Gather arguments
    parser = argparse.ArgumentParser(description="Setup a new workspace for a RCR patch set.")
    parser.add_argument('working_directory', type=Path)
    parser.add_argument(
        "--release",
        required=True,
        help="Bazel 'ros' module release version, e.g. rolling.2026-01-21",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite the workspace directory if it exists",
    )
    args = parser.parse_args()

    with yaspin(text="Setting up new workspace", color="cyan") as sp:

        sp.write("> Finding latest patched version for release")
        old_patch = _find_latest_patch(args.working_directory, args.release)
        if old_patch is None:
            raise RuntimeError("Release {0} does not exist or has no patches.".format(args.release))
        sp.write("> Found patch {0}".format(old_patch))

        sp.write("> Scanning target release for packages")
        old_dir = args.working_directory / "modules" / "ros" / old_patch
        old_packages = scan_module_for_dependencies(old_dir)
        old_packages["ros"] = args.release
        sp.write("> Found {0} packages".format(len(old_packages)))

        new_patch = increment_version(old_patch)
        sp.write("> New patch with name {0}".format(new_patch))

        workspace_dir = args.working_directory / "workspace" / new_patch
        try:
            workspace_dir.mkdir(parents=True)
        except FileExistsError:
            if not args.overwrite:
                raise RuntimeError("Patch directory {0} already exists.".format(workspace_dir))
        sp.write("> Created new patch directory {0}".format(workspace_dir.relative_to(args.working_directory)))


        _setup_patch_workspace(workspace_dir, "workspace", new_patch, old_packages)

        # sp.write("> Scanning target release for packages")
        # base_packages = _scan_release_for_packages(base_dir)
        # base_packages["ros"] = args.release
        # sp.write("> Found {0} packages".format(len(base_packages)))

        # sp.write("> Checking for previous patch release")
        # maybe_old_dir = _find_previous_release(args.working_directory, args.release, args.from_distro)
        # prev_packages = {}
        # if maybe_old_dir is None:   
        #     sp.write("> No suitable previous patch release found")
        # else:
        #     sp.write("> Found previous patch release: {0}".format(maybe_old_dir.name))
        #     sp.write("> Scanning previous patch release for packages")
        #     prev_packages = _scan_release_for_packages(maybe_old_dir)
        #     prev_packages["ros"] = maybe_old_dir.name
        #     sp.write("> Found {0} packages".format(len(prev_packages)))

        # # As we migrate packages, we need to keep track of the old and new versions
        # # to update the dependencies after the migration has occured.
        # sp.write("> Migrating modules")
        # old_packages = {}
        # new_packages = {}
        # for package_name, package_base_version in base_packages.items():
        #     sp.write("> Processing {0}".format(package_name))
        #     package_dir = args.working_directory / "modules" / package_name
        #     if package_name in prev_packages.keys():
        #         package_new_version = _package_migrate(
        #             package_dir,
        #             package_base_version,
        #             prev_packages[package_name]
        #         )
        #         sp.write("  + Migrated {0}@{1} -> {0}@{2}".format(
        #             package_name, prev_packages[package_name], package_new_version))
        #         old_packages[package_name] = prev_packages[package_name]
        #     else:
        #         package_new_version = _package_create(
        #             package_dir,
        #             package_base_version
        #         )
        #         sp.write("  + Created new package {0}@{1}".format(
        #             package_name, package_new_version))
        #         old_packages[package_name] = package_base_version
        #     regenerate_integrity_hashes(package_dir / package_new_version)
        #     sp.write("  + Regenerated integrity hashes for {0}@{1}".format(
        #         package_name, package_new_version))
        #     new_packages[package_name] = package_new_version


        # # Run through all MODULE.bazel files replacing any bazel_dep calls to old
        # # package versions with the new package versions.
        # sp.write("> Updating dependencies")
        # for package_name, package_new_version in new_packages.items():
        #     module_file = args.working_directory / "modules" / package_name / package_new_version / "MODULE.bazel"
        #     _update_package_dependencies(module_file, package_name, old_packages, new_packages)
        #     sp.write("  + Updated dependencies for {0}@{1}".format(
        #         package_name, package_new_version))

        sp.ok("✔")


if __name__ == "__main__":
    main()