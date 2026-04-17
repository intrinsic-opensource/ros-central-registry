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
import re
from pathlib import Path
from termcolor import COLORS, HIGHLIGHTS
from typing import Optional, Dict, List, Set
from yaspin import yaspin
from yaspin.spinners import Spinners
from bazelflore.utils.bzlmod import increment_version
from bazelflore.utils.bzlmod import scan_module_for_dependencies
from bazelflore.utils.bzlmod import find_latest_patch
from bazelflore.utils.copyright import get_copyright_header

BAZEL_VERSION = "9.0.2"

VARIANTS = {
    "core" : "ros_core",
    "base" : "ros_base",
    "desktop" : "desktop",
    "desktop_full" : "desktop_full",
    "perception" : "perception",
    "simulation" : "simulation",
}           

def calculate_packages_for_variant(module_dir : Path, start_name: str, start_version: Path) -> Set[str]:
    """
    Performs a BFS traversal starting from the given module name and version to collect
    all transitive dependencies.
    """
    visited = set()
    queue = [(start_name, module_dir / start_name / start_version / 'MODULE.bazel')]
    while queue:
        current_name, current_file = queue.pop(0)
        if current_name in visited:
            continue
        visited.add(current_name)
        if not current_file.exists():
            continue
        with open(current_file, "r") as f:
            content = f.read()            
            deps = re.findall(r'bazel_dep\(name\s*=\s*"([^"]+)"\s*,\s*version\s*=\s*"([^"]+)"\)', content)
            for next_name, next_version in deps:
                next_file = module_dir / next_name / next_version / 'MODULE.bazel'
                if next_file.exists():
                    if next_name not in visited:
                        queue.append((next_name, next_file))
    return sorted(list(visited))

def _setup_workspace(
    workspace_dir: Path,
    module_name: str,
    module_version: str,
    variants: Dict[str, List[str]],
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
        f.write(BAZEL_VERSION)

    # Create a BUILD.bazel file
    build_dot_bazel_file = workspace_dir / "BUILD.bazel"
    with open(build_dot_bazel_file, 'w') as f:
        f.write(get_copyright_header())

    # Create configuration files for each ROS variant.
    for variant_name, variant_packages in variants.items():
        configuration_file = workspace_dir / f"ros-{variant_name}.txt"
        with open(configuration_file, 'w') as f:
            f.write("\n".join([f"@{p}//..." for p in variant_packages]))

    # Create a .bazelrc file
    dot_bazelrc_file = workspace_dir / ".bazelrc"
    with open(dot_bazelrc_file, 'w') as f:
        repo_args = "\n".join(
            [
                f"\t--repo=@{p} \\"
                for p in sorted(packages.keys())
                if vendor is None or p in vendor
            ]
        )
        variant_args = "\n".join(
            [
                f"common:{k} --target_pattern_file=ros-{k}.txt"
                for k in variants.keys()
            ]
        )
        f.write(get_copyright_header())
        f.write("""
common --check_direct_dependencies=off
common --registry=file://%workspace%/../..  --registry=https://bcr.bazel.build
common --remote_cache=https://storage.googleapis.com/intrinsic-opensource-buildcache
common --remote_upload_local_results=false
common --remote_cache_compression=true
common --test_env=ROS_DISTRO="rolling"
common --test_env=ROS_HOME=".ros"
common --test_env=RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
common --incompatible_default_to_explicit_init_py
common --incompatible_strict_action_env
test --sandbox_default_allow_network=false
build --@protobuf//bazel/toolchains:prefer_prebuilt_protoc
{0}
vendor --vendor_dir=vendor {1}
""".format(variant_args, repo_args))

    # Create a VENDOR.bazel file
    vendor_dot_bazel = workspace_dir / "vendor" / "VENDOR.bazel"
    vendor_dot_bazel.parent.mkdir(parents=True, exist_ok=True)
    with open(vendor_dot_bazel, 'w') as f:
        for package in sorted(packages.keys()):
            if vendor is None or package in vendor:
                f.write("pin(\"@@{0}\")\n".format(package))

    # Create a MODULE.bazel file
    module_dot_bazel_files = workspace_dir / "MODULE.bazel"
    with open(module_dot_bazel_files, 'w') as f:
        f.write(get_copyright_header())
        f.write(f"""module(
    name = "{module_name}",
    version = "{module_version}",
)

# BCR deps
bazel_dep(name = "aspect_rules_py", version = "1.11.2")
bazel_dep(name = "bazel_skylib", version = "1.9.0")
bazel_dep(name = "cmake_configure_file", version = "0.1.7")
bazel_dep(name = "google_benchmark", version = "1.9.5")
bazel_dep(name = "googletest", version = "1.17.0.bcr.2")
bazel_dep(name = "llvm", version = "0.7.1")
bazel_dep(name = "platforms", version = "1.0.0")
bazel_dep(name = "rules_cc", version = "0.2.17")
bazel_dep(name = "rules_pkg", version = "1.2.0")
bazel_dep(name = "rules_python", version = "1.9.0")
bazel_dep(name = "rules_qt", version = "0.0.6")
bazel_dep(name = "rules_rs", version = "0.0.56")
bazel_dep(name = "rules_rust", version = "0.69.0")
bazel_dep(name = "rules_shell", version = "0.7.1")
bazel_dep(name = "toolchains_llvm", version = "1.6.0")

# Setup C and C++
llvm = use_extension("@toolchains_llvm//toolchain/extensions:llvm.bzl", "llvm")
llvm.toolchain(llvm_version = "20.1.7")
use_repo(llvm, "llvm_toolchain")

register_toolchains("@llvm_toolchain//:all")

# Setup python
python = use_extension("@rules_python//python/extensions:python.bzl", "python")
python.toolchain(
    is_default = True,
    python_version = "3.12",
)

# Setup pip
pip_ros = use_extension("@rosdistro//python:defs.bzl", "pip_ros")
use_repo(pip_ros, "pip_ros")

# Setup rust
rust = use_extension("@rules_rust//rust:extensions.bzl", "rust")
rust.toolchain(
    edition = "2021",
    versions = ["1.85.0"],
)

# RCR deps
""")
        for package in sorted(packages.keys()):
            f.write("bazel_dep(name = \"{0}\", version = \"{1}\")\n".format(package, packages[package]))
        f.write("\n# include(\":dev.MODULE.bazel\")")

    # Create a dev.MODULE.bazel file that forces a local_path_override fto
    # the vendored package. This allows you to iterate and test as you go.
    dev_module_dot_bazel_file = workspace_dir / "dev.MODULE.bazel"
    with open(dev_module_dot_bazel_file, 'w') as f:
        f.write(get_copyright_header())
        for package in sorted(packages.keys()):
            f.write("""
local_path_override(
    module_name = \"{0}\",
    path = \"./vendor/{0}+\",
)
""".format(package))

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
        ref_packages["rosdistro"] = args.release
        sp.write("> Found {0} packages".format(len(ref_packages)))

        # Create configuration files for each ROS variant.
        sp.write("> Detecting variants")
        variants = {}
        for variant_name, variant_package in VARIANTS.items():
            variants[variant_name] = calculate_packages_for_variant(
                modules_dir, variant_package, ref_packages[variant_package])
            sp.write("    {0}: {1} packages".format(variant_name, len(variants[variant_name])))

        # Set up development workspace.
        sp.write("> Setting up workspace {0} for development".format(args.release))
        ref_workspace_dir = args.working_directory / "workspace" / args.release
        ref_count = _setup_workspace(
            workspace_dir=ref_workspace_dir,
            module_name="rcr",
            module_version=args.release,
            variants=variants,
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
        old_packages["rosdistro"] = args.release
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
            variants=variants,
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