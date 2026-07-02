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
Setup a Bazel workspace for testing ROS distributions in CI.
"""

import argparse
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Set

def scan_module_for_dependencies(
    module_dot_bazel: Path,
    modules_path: Path,
    include_bcr: bool = False,
    include_rcr: bool = True,
) -> Dict[str, str]:
    """
    Returns a list of package names and their versions.
    """
    packages = {}
    with open(module_dot_bazel, "r") as f:
        content = f.read()
        matches = re.findall(
            r'bazel_dep\(\s*name\s*=\s*"([^"]+)"\s*,\s*version\s*=\s*"([^"]+)"',
            content,
        )
        for name, version in matches:
            is_rcr_module = (modules_path / name).exists()
            if (include_rcr and is_rcr_module) or (
                include_bcr and not is_rcr_module
            ):
                packages[name] = version
    return packages


VARIANTS = {
    "core": "ros_core",
    "base": "ros_base",
    "desktop": "desktop",
    "desktop_full": "desktop_full",
    "perception": "perception",
    "simulation": "simulation",
}

BAZELRC_TEMPLATE = """# Copyright 2026 Open Source Robotics Foundation, Inc.
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

## COMMON OPTIONS

# This suppresses warnings that bazel emits when two dependencies rely
# on different versions of a single module. This is what allows us to
# load 0.0.0 versions of RCR package and have bazel resolve the correct
# package for the ROS distribution we are using.
common --check_direct_dependencies=off

# This tells Bazel to look locally for RCR modules at the root of this
# project, additional "staged" BCR modules in the bcr_staging folder
# in this workspace, and ultimately at the BCR for the rest.
common --registry=file://%workspace%/..             \\
       --registry=file://%workspace%/../bcr_staging \\
       --registry=https://bcr.bazel.build

# This provides read-only access to a shared Bazel remote cache offered
# by Intrinsic. It helps speed up fresh checkouts from upstream.
common --remote_cache=https://storage.googleapis.com/intrinsic-opensource-buildcache
common --remote_upload_local_results=false
common --remote_cache_compression=true

# Critical ROS environment variables needed at test-time.
common --test_env=ROS_DISTRO="{distro}"
common --test_env=ROS_HOME=".ros"
common --test_env=RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
common --test_env=LD_LIBRARY_PATH=lib

# Critical ROS environment variables needed at run-time.
common --run_env=ROS_DISTRO="{distro}"
common --run_env=ROS_HOME=".ros"
common --run_env=RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
common --run_env=LD_LIBRARY_PATH=lib

# Propagate select X11 variables through to the test sandbox, so that
# any test that uses the display has access to it.
common --test_env=DISPLAY
common --test_env=XAUTHORITY
common --test_env=WAYLAND_DISPLAY

# Our approach to message generation works on a per-message dependency
# chain. We need to be able to merge messages into 
common --incompatible_default_to_explicit_init_py

# This restricts the environment variables available to build actions, 
# keeping the build environment more hermetic.
common --incompatible_strict_action_env

## BUILD OPTIONS

# Ensure that we use toolchains_llvm instead of the host toolchain.
build --action_env="BAZEL_DO_NOT_DETECT_CPP_TOOLCHAIN=1"

# Our bazel distribution provides conversion utilities for transforming
# ROS messages to .proto files. This prevent protobuf from trying to
# recompile itself in response to environment changes.
build --@protobuf//bazel/toolchains:prefer_prebuilt_protoc

# This tells our hermetic LLVM compiler to stub some GCC functions that
# are used in several places around the ROS Bazel workspace/
build --@llvm//config:experimental_stub_libgcc_s=True

# Include the realtime library when linking (until fixed in FastDDS).
build --linkopt="-lrt"

## TEST OPTIONS

# This restricts our test sandboxes from accessing the network, which is
# important if you want to prevent destructive interference on the ROS
# messaging system between tests running in parallel
test --sandbox_default_allow_network=false

# CONFIG OPTIONS FOR CI WORKER
common --remote_cache=https://storage.googleapis.com/intrinsic-opensource-buildcache
common --remote_upload_local_results=true
common --remote_cache_compression=true
common --google_default_credentials
common --flaky_test_attempts=5

# Our CI workers struggle with test parallelism.
test --local_test_jobs=1

## CONFIG OPTIONS FOR VARIANTS
common:core --target_pattern_file=ros-core.txt
common:base --target_pattern_file=ros-base.txt
common:desktop --target_pattern_file=ros-desktop.txt
common:desktop_full --target_pattern_file=ros-desktop_full.txt
common:perception --target_pattern_file=ros-perception.txt
common:simulation --target_pattern_file=ros-simulation.txt
"""

def get_copyright_header() -> str:
    return """# Copyright 2026 Open Source Robotics Foundation, Inc.
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

def calculate_packages_for_variant(
    module_dir: Path, start_name: str, start_version: str
) -> List[str]:
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
            deps = re.findall(
                r'bazel_dep\(name\s*=\s*"([^"]+)"\s*,\s*version\s*=\s*"([^"]+)"\)',
                content,
            )
            for next_name, next_version in deps:
                next_file = module_dir / next_name / next_version / 'MODULE.bazel'
                if next_file.exists():
                    if next_name not in visited:
                        queue.append((next_name, next_file))
    visited.discard("rosdistro")
    return sorted(list(visited))

def main():
    parser = argparse.ArgumentParser(
        description="Setup a Bazel workspace for testing ROS distributions in CI."
    )
    parser.add_argument(
        "--release",
        required=True,
        help="Bazel 'ros' module release version, e.g. lyrical.2026-06-08.rcr.1",
    )
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=Path("workspace"),
        help="Directory to create the workspace in",
    )
    args = parser.parse_args()

    # Locate directories
    workspace_root = Path(
        os.environ.get("BUILD_WORKSPACE_DIRECTORY", ".")
    ).resolve()
    modules_dir = workspace_root / "modules"
    ros_module_dir = modules_dir / "ros" / args.release

    if not ros_module_dir.exists():
        print(f"Error: ROS release module path {ros_module_dir} does not exist.", file=sys.stderr)
        sys.exit(1)

    # Scan release for dependencies
    print(f"Scanning ROS release '{args.release}' for packages...")
    packages = scan_module_for_dependencies(
        ros_module_dir / "MODULE.bazel", modules_dir
    )
    # Add ros and rosdistro to match the behavior of workspace_setup.py
    packages["ros"] = args.release
    packages["rosdistro"] = args.release
    print(f"Found {len(packages)} RCR packages in release.")

    # Detect variants
    variants: Dict[str, List[str]] = {}
    for variant_name, variant_package in VARIANTS.items():
        if variant_package not in packages:
            print(f"Warning: Variant package {variant_package} not found in release.")
            continue
        variants[variant_name] = calculate_packages_for_variant(
            modules_dir, variant_package, packages[variant_package]
        )
        print(f"  {variant_name}: {len(variants[variant_name])} packages")

    # Create workspace directory
    target_workspace = (workspace_root / args.workspace_dir).resolve()
    target_workspace.mkdir(parents=True, exist_ok=True)
    print(f"Creating workspace at {target_workspace}...")

    # Write BUILD.bazel
    with open(target_workspace / "BUILD.bazel", "w") as f:
        f.write(get_copyright_header())

    # Write ros-<variant>.txt files
    for variant_name, variant_packages in variants.items():
        with open(target_workspace / f"ros-{variant_name}.txt", "w") as f:
            f.write("\n".join([f"@{p}//..." for p in variant_packages]))

    # Write MODULE.bazel
    with open(target_workspace / "MODULE.bazel", "w") as f:
        f.write(get_copyright_header())
        f.write("""module(
    name = "rcr-workspace",
)

# BCR deps

bazel_dep(name = "aspect_rules_py", version = "1.11.2")
bazel_dep(name = "llvm", version = "0.8.6")
bazel_dep(name = "platforms", version = "1.1.0")
bazel_dep(name = "protobuf", version = "35.0-rc1")
bazel_dep(name = "rules_cc", version = "0.2.18")
bazel_dep(name = "rules_go", version = "0.60.0")
bazel_dep(name = "rules_python", version = "1.9.0")
bazel_dep(name = "rules_rs", version = "0.0.56")
bazel_dep(name = "rules_rust", version = "0.69.0")
bazel_dep(name = "rules_shell", version = "0.8.0")

## RCR deps

""")
        f.write(f'bazel_dep(name = "ros", version = "{args.release}")\n\n')
        for name in sorted(packages.keys()):
            if name != "ros":
                f.write(f'bazel_dep(name = "{name}", version = "{packages[name]}")\n')

        f.write("""
## BOOTSTRAP

# Register our hermetic compiler (clang)
register_toolchains("@llvm//toolchain:all")

# Python / pip

python = use_extension("@rules_python//python/extensions:python.bzl", "python")
python.toolchain(
    is_default = True,
    python_version = "3.12",
)

pip_ros = use_extension("@rosdistro//python:defs.bzl", "pip_ros")
use_repo(pip_ros, "pip_ros")

# Rust / crates

rust = use_extension("@rules_rust//rust:extensions.bzl", "rust")
rust.toolchain(
    edition = "2021",
    versions = ["1.85.0"],
)
""")

    # Write .bazelrc
    distro = args.release.split(".")[0]
    with open(target_workspace / ".bazelrc", "w") as f:
        f.write(BAZELRC_TEMPLATE.format(distro=distro))

    print("Workspace setup complete.")

if __name__ == "__main__":
    main()
