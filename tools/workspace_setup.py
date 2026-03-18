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
from typing import Optional, Dict, List
from yaspin import yaspin
from yaspin.spinners import Spinners
from bazelflore.utils.bzlmod import increment_version
from bazelflore.utils.bzlmod import scan_module_for_dependencies
from bazelflore.utils.bzlmod import find_latest_patch
from bazelflore.utils.copyright import get_copyright_header

def _setup_workspace(
    workspace_dir: Path,
    module_name: str,
    module_version: str,
    packages: Dict[str, str],
    vendor: Optional[List[str]] = None,
    skip_if_exists: bool = False,
):
    """
    Setup a new workspace for RCR development at <workspace_dir>. This workspace
    is intended to be used for local development and testing of RCR packages.
    """

    # If the workspace already exists and we are skipping, return the number of packages.
    if workspace_dir.exists():
        if skip_if_exists:
            return len(packages)
        raise RuntimeError("Patch directory {0} already exists.".format(workspace_dir))
    else:
        workspace_dir.mkdir(parents=True)

    # Create a .bazelversion file
    dot_bazelversion_file = workspace_dir / ".bazelversion"
    with open(dot_bazelversion_file, 'w') as f:
        f.write("9.0.0")

    # Create a .bazelignore file
    # dot_bazelignore_file = workspace_dir / ".bazelignore"
    # with open(dot_bazelignore_file, 'w') as f:
    #     f.write("base\n")
    #     f.write("feat\n")

    # Create a .bazelrc file
    dot_bazelrc_file = workspace_dir / ".bazelrc"
    with open(dot_bazelrc_file, 'w') as f:
        repo_args = [
            f"\t--repo=@{p} \\" for p in sorted(packages.keys())
            if vendor is None or p in vendor
        ]
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
            if vendor is None or package in vendor:
                f.write("pin(\"@@{0}\")\n".format(package))

    # Create a distribution.txt file
    distribution_txt_file = workspace_dir / "distribution.txt"
    with open(distribution_txt_file, 'w') as f:
        for package in sorted(packages.keys()):
            if vendor is None or package in vendor:
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

    # Return how many packages were vendored.
    return len(vendor) if vendor is not None else len(packages)

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

    with yaspin(text="Setting up new workspace", color="cyan") as sp:
        # REF: rolling.2026-01-21.        (bare release without patches)
        # OLD: rolling.2026-01-21,bcr.2   (what to start with)
        # NEW: rolling.2026-01-21.bcr.3   (what we are adding)
        modules_dir = args.working_directory / "modules"

        # Find all the packages in the REF release and make sure there is a vendored workspace.
        # in place to use a reference when calculating patch sets.
        sp.write("> Scanning REF release '{0}' for packages".format(args.release))
        ref_dir = args.working_directory / "modules" / "ros" / args.release
        ref_packages = scan_module_for_dependencies(ref_dir / 'MODULE.bazel', modules_dir)
        ref_packages["ros"] = args.release
        sp.write("> Found {0} packages".format(len(ref_packages)))
        sp.write("> Setting up workspace {0} for development".format(args.release))
        ref_workspace_dir = args.working_directory / "workspace" / args.release
        ref_count = _setup_workspace(
            workspace_dir=ref_workspace_dir,
            module_name="rcr",
            module_version=args.release,
            packages=ref_packages,
            skip_if_exists=True
        )
        sp.write("> REF release {0} has {1} vendored packages".format(args.release, ref_count))

        # We need to find the latest patch set for the given release, as we will be building
        # on top of these patches to create our new patch set.
        sp.write("> Finding OLD release")
        old_patch = find_latest_patch(args.working_directory, args.release)
        if old_patch is None:
            raise RuntimeError("Please bootstrap {0} to create {0}.rcr.0".format(args.release))
        sp.write("> Found OLD release {0}".format(old_patch))
        sp.write("> Scanning OLD release for packages")
        old_dir = args.working_directory / "modules" / "ros" / old_patch
        old_packages = scan_module_for_dependencies(old_dir / 'MODULE.bazel', modules_dir)
        old_packages["ros"] = args.release
        sp.write("> Found {0} packages in OLD release".format(len(old_packages)))

        # Find the new patch version.
        sp.write("> Calculating NEW patch")
        new_patch = increment_version(old_patch)
        sp.write("> New patch with name {0}".format(new_patch))
        if args.packages is None:
            sp.write("> No packages specified, using all packages from previous patch release")
        else:
            sp.write("> Including packages: {0}".format(", ".join(args.packages)))
        sp.write("> Setting up workspace {0} for development".format(new_patch))
        new_workspace_dir = args.working_directory / "workspace" / new_patch
        new_count = _setup_workspace(
            workspace_dir=new_workspace_dir,
            module_name="rcr",
            module_version=new_patch,   
            packages=old_packages,      # We build off the OLD release.
            vendor=args.packages,       # Only vendor these specific packages
            skip_if_exists=False,       # This patch set must exist
        )
        sp.write("> NEW patch '{0}' has {1} vendored packages".format(new_patch, new_count))
        sp.write("******************************** IMPORTANT **********************************")
        sp.write("* YOU MUST DO THE FOLLOWING BEFORE CONTINUING...")
        sp.write("* 1. Run 'bazel vendor' in workspace {0}".format(ref_workspace_dir.relative_to(args.working_directory)))
        sp.write("* 2. Run 'bazel vendor' in workspace {0}".format(new_workspace_dir.relative_to(args.working_directory)))
        sp.write("*****************************************************************************")
        sp.ok("✔")


if __name__ == "__main__":
    main()