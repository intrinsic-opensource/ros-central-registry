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

# Use a shared cache for bazel builds, which speeds up CI builds.
common --remote_cache=https://storage.googleapis.com/intrinsic-opensource-buildcache
common --remote_cache_compression=true
common --remote_upload_local_results=false

# CI specific options for remote caching and credentials.
common:ci --remote_upload_local_results=true
common:ci --google_default_credentials

# We support the following variants of ROS distributions. This allows you
# to build or test the entire variant with a --config flag.
{variants}

# Automatically pick configurations for the platform running bazel.
# For example, macos on macos, linux on linux, etc.
common --enable_platform_specific_config

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

# ld64's two-level namespace requires every undefined symbol in a .dylib to
# be resolvable among the libraries passed directly to its link command.
# Our rosidl-generated cc_shared_library targets commonly need symbols from
# libraries that are several dynamic_deps hops away, which the ELF loader on
# Linux resolves transitively at process load time with no issue. On Darwin
# we fall back to flat-namespace symbol lookup at load time instead.
build:macos --linkopt=-Wl,-undefined,dynamic_lookup
build:macos --host_linkopt=-Wl,-undefined,dynamic_lookup

# The Apple SDK framework directories contain module.modulemap files that
# activate Clang's implicit module system once a Frameworks directory is in
# scope (as it now implicitly is, via --sysroot). Libraries like abseil-cpp
# include CoreFoundation headers without declaring a module dependency, which
# the module system rejects. -fno-implicit-module-maps is a clang driver flag
# (not a cc1 flag); pass it directly without -Xclang.
build:macos --copt=-fno-implicit-module-maps
build:macos --host_copt=-fno-implicit-module-maps

# Apple SDK framework headers use the CF_ENUM() macro, which expands to a forward
# enum declaration inside a typedef -- a Clang extension that Clang supports but
# warns about starting with Clang 22 (-Welaborated-enum-base). Suppress it so
# CoreFoundation/CFBase.h and friends compile cleanly with the hermetic toolchain.
build:macos --copt=-Wno-elaborated-enum-base
build:macos --host_copt=-Wno-elaborated-enum-base

# handle unresolved symlinks in frameworks
build:macos --experimental_allow_unresolved_symlinks
build:macos --nested_set_depth_limit=5000


## TEST OPTIONS

# This restricts our test sandboxes from accessing the network, which is
# important if you want to prevent destructive interference on the ROS
# messaging system between tests running in parallel. On Linux, this still
# permits loopback, so DDS discovery over loopback keeps working. macOS's has
# profile only carves out plain "localhost:*" traffic, not UDP
# multicast groups, which is what FastRTPS's discovery protocol actually
# uses -- so multi-node/multi-context tests (e.g. rcl_action's graph tests)
# hang until they hit their own timeout under the sandbox. For the tests that
# need real multicast discovery, we use "no-sandbox" + "exclusive" tags on
# just the handful of tests that need real multicast discovery.
test --sandbox_default_allow_network=false

# CI specific quirks
test:ci --flaky_test_attempts=5
test:ci --local_test_jobs=1
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

# Extend the hermetic macOS SDK sysroot (@llvm//extensions:osx.bzl) with the
# frameworks our dependencies need: IOKit for fastdds (utils/Host.cpp,
# resolved version 3.4.2 here) and some zenoh-c crates, plus everything
# opencv needs for its macOS highgui/videoio backends.
#
# The osx.bzl extension stages System/Library/Frameworks/<Name>.framework/*
# for each requested name -- nothing more. It does *not* follow the C/ObjC
# #include graph, so anything a requested framework's headers pull in via
# <OtherFramework/Header.h> has to be requested too, or the compile fails
# with "file not found" partway through some umbrella header. This list was
# derived by cloning the actual SDK headers (github.com/phracker/MacOSX-SDKs)
# and tracing every #include reachable from the four umbrella headers opencv
# actually compiles against on macOS: Cocoa.h and AVFoundation.h (imported
# directly by modules/highgui/src/window_cocoa.mm and
# modules/videoio/src/cap_avfoundation_mac.mm), plus AppKit.h and CoreData.h
# (which Cocoa.h itself unconditionally imports) -- rather than discovering
# each missing framework one CI failure at a time. Accelerate, Cocoa,
# CoreGraphics, CoreMedia, CoreVideo, and QuartzCore additionally match the
# exact "-framework" linkopts the "opencv" module (4.13.0.bcr.5 as resolved
# here) declares for @platforms//os:macos on its ocv.3rdparty.cocoa and
# ocv.3rdparty.avfoundation objc_library targets.
#
# Two things complicate a pure "list every framework reachable" approach:
#
# 1. Some frameworks referenced via <X/Y.h> are actually *nested*,
#    self-contained copies bundled inside another framework's own
#    Frameworks/ subdirectory (e.g. CoreServices.framework/Frameworks/
#    {AE,CarbonCore,DictionaryServices,FSEvents,LaunchServices,Metadata,
#    OSServices,SearchKit,SharedFileList}.framework, and similarly under
#    ApplicationServices.framework/Frameworks/{ATS,ATSUI,ColorSyncLegacy,
#    HIServices,LangAnalysis,PrintCore,QD,SpeechSynthesis}.framework).
#    Clang resolves these via its framework-relative "subframework" header
#    search without needing them in this list at all, as long as the
#    *enclosing* framework (CoreServices / ApplicationServices) is present.
#
# 2. A handful of those same nested entries are instead *symlinks* out to a
#    sibling top-level framework rather than self-contained copies: e.g.
#    AVFoundation.framework/Versions/A/Frameworks/AVFAudio.framework and
#    ApplicationServices.framework/.../Frameworks/{ColorSync,CoreText,
#    ImageIO}.framework. Bazel's sandbox refuses to stage a dangling symlink
#    to a framework that wasn't requested ("file type ... is not
#    supported") for *any* action depending on the sysroot as a whole, not
#    just ones that touch the framework in question -- so AVFAudio,
#    ColorSync, CoreText, and ImageIO all have to be requested explicitly
#    even though nothing here calls them directly, purely to give those
#    symlinks a real target. This isn't limited to whole framework bundles
#    either: ApplicationServices.../Frameworks/ATS.framework/Versions/A/
#    Resources/{libFontParser,libhvf,libTrueTypeScaler,libType1Scaler}.tbd
#    are themselves symlinks out to System/Library/PrivateFrameworks/
#    FontServices.framework, several directory levels deeper than the
#    top-level bundle symlinks above -- hence FontServices below, which is
#    otherwise never referenced by name anywhere in this file. A full
#    recursive symlink audit of every framework in this list (cloning
#    github.com/phracker/MacOSX-SDKs and walking each requested bundle for
#    symlinks pointing outside the requested set) turned up FontServices as
#    the only remaining gap.
#
# CoreAudioTypes is its own top-level framework for a related reason:
# CoreAudio.framework/Headers/CoreAudioTypes.h is just a one-line forward to
# <CoreAudioTypes/CoreAudioTypes.h> in the real SDK.
#
# The default framework list (everything below except IOKit and the ones
# explained above) comes from the "llvm" module itself; since
# osx.frameworks(...) tags from every module in the graph get merged into
# one list, but that "llvm"-provided default only applies when nobody sets
# the tag at all -- as soon as any module (including this one) sets it, the
# default drops out, so we have to repeat it here rather than append.
osx = use_extension("@llvm//extensions:osx.bzl", "osx")
osx.frameworks(
    names = [
        "AVFoundation",
        "Accelerate",
        "Cocoa",
        "CoreGraphics",
        "CoreImage",
        "CoreMedia",
        "CoreVideo",
        "Foundation",
        "OpenGL",
        "QuartzCore",
    ],
)

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
    variants = "\n".join(
        f"common:{variant_name} --target_pattern_file=ros-{variant_name}.txt"
        for variant_name in VARIANTS
    )
    with open(target_workspace / ".bazelrc", "w") as f:
        f.write(BAZELRC_TEMPLATE.format(distro=distro, variants=variants))

    print("Workspace setup complete.")

if __name__ == "__main__":
    main()
