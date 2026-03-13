#!/usr/bin/env python3
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

"""Generates BUILD.bazel files for ROS 2 interface definitions.

This tool iterates through subdirectories of a specified directory, locating
ROS 2 interface files (.msg, .srv, .action, .idl) and generating the
corresponding BUILD.bazel files with appropriate Bazel rules and dependencies.

Usage:
    python interface_autogen.py <directory> [subdirectory ...]

Examples:
    # Process all subdirectories in ./submodules:
    python interface_autogen.py ./submodules

    # Process only specific subdirectories:
    python interface_autogen.py ./submodules geometry_msgs sensor_msgs
"""

import argparse
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

# File extension to Bazel rule mapping.
EXTENSION_TO_RULE = {
    ".msg": "ros_message",
    ".srv": "ros_service",
    ".action": "ros_action",
}

# Extensions we care about.
INTERFACE_EXTENSIONS = set(EXTENSION_TO_RULE.keys())

# Copyright header for generated BUILD.bazel files.
COPYRIGHT_HEADER = """\
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

# These are the the types supported by the ROS 2 IDL compiler. Anything
# else is considered a custom type and must be a dependency.
PRIMITIVE_TYPES = {
    'bool',
    'byte',
    'char',
    'float32',
    'float64',
    'int8',
    'uint8',
    'int16',
    'uint16',
    'int32',
    'uint32',
    'int64',
    'uint64',
    'string',
    'wstring'
}

def parse_type(type_str):
    """Extract the base type from a ROS 2 type string.

    Strips array suffixes like [], [N], and bounded types like
    string<=128 or sequence<type, N>.

    Args:
        type_str: The raw type string from an interface file.

    Returns:
        The base type string with array/bound modifiers removed.
    """
    # Remove array suffixes: Type[], Type[N], Type[<=N]
    base_type = re.sub(r"\[.*\]$", "", type_str)
    # Remove bounded string suffixes: string<=N
    base_type = re.sub(r"<=\d+$", "", base_type)
    return base_type.strip()


def is_constant_definition(line):
    """Check if a line is a constant definition (contains '=').

    Constant definitions in ROS 2 interface files follow the pattern:
        <type> <NAME> = <value>

    Args:
        line: A stripped, non-comment line from an interface file.

    Returns:
        True if the line defines a constant, False otherwise.
    """
    # Split into tokens; if there's a '=' after the name, it's a constant.
    parts = line.split()
    if len(parts) >= 3 and "=" in parts[2:]:
        return True
    # Also check for '=' attached to the name: "uint8 FOO=1"
    if len(parts) >= 2 and "=" in parts[1]:
        return True
    return False


def map_package_interfaces(package_root):
    """Scan the package root for all interface files and map Type -> RelDir."""
    interface_map = {}
    for root, dirs, files in os.walk(package_root):
        # Skip directories containing "ros1"
        if "ros1" in root:
            continue
        for filename in files:
            name, ext = os.path.splitext(filename)
            if ext in INTERFACE_EXTENSIONS:
                rel_dir = os.path.relpath(root, package_root)
                if rel_dir == ".":
                    rel_dir = ""
                interface_map[name] = rel_dir
    return interface_map


def detect_rosidl_runtime(package_dir):
    """Detect which ROS IDL runtime to use by parsing MODULE.bazel.

    Args:
        package_dir: Root directory of the package.

    Returns:
        The Bazel label for the runtime (e.g., "@rosidl_core_runtime//:defs.bzl").
    """
    module_bazel = os.path.join(package_dir, "MODULE.bazel")
    if os.path.exists(module_bazel):
        with open(module_bazel, "r") as f:
            content = f.read()
            # Look for bazel_dep(name = "rosidl_core_runtime"
            if re.search(r'bazel_dep\(\s*name\s*=\s*["\']rosidl_core_runtime["\']', content):
                return "@rosidl_core_runtime//:defs.bzl"
            # Look for bazel_dep(name = "rosidl_default_runtime"
            if re.search(r'bazel_dep\(\s*name\s*=\s*["\']rosidl_default_runtime["\']', content):
                return "@rosidl_default_runtime//:defs.bzl"
            # Look for bazel_dep(name = "rosidl_default_generators"
            if re.search(r'bazel_dep\(\s*name\s*=\s*["\']rosidl_default_generators["\']', content):
                return "@rosidl_default_generators//:defs.bzl"

    # Default to rosidl_default_runtime if unknown.
    return "@rosidl_default_runtime//:defs.bzl"


def extract_dependencies(filepath, package_root, interface_map):
    """Parse a ROS 2 interface file and extract external and internal dependencies.

    Reads the interface file line by line, skipping comments, blank lines,
    section separators (---), and constant definitions.

    Args:
        filepath: Path to the interface file (.msg, .srv, .action, .idl).
        package_root: Root directory of the ROS package.
        interface_map: Dict mapping interface names to their relative directories.

    Returns:
        A sorted list of unique dependency strings.
    """
    return extract_dependencies_v2(filepath, package_root, interface_map, "", {})


def extract_dependencies_v2(filepath, package_root, interface_map, current_pkg_name, global_interface_map):
    """Parse a ROS 2 interface file and extract external and internal dependencies.

    Reads the interface file line by line, skipping comments, blank lines,
    section separators (---), and constant definitions.

    Args:
        filepath: Path to the interface file (.msg, .srv, .action, .idl).
        package_root: Root directory of the ROS package.
        interface_map: Dict mapping interface names to their relative directories.
        current_pkg_name: Name of the current package being processed.
        global_interface_map: Dict mapping package names to their interface maps.

    Returns:
        A sorted list of unique dependency strings.
    """
    deps = set()
    current_dir = os.path.dirname(filepath)
    rel_current_dir = os.path.relpath(current_dir, package_root)
    if rel_current_dir == ".":
        rel_current_dir = ""

    with open(filepath, "r") as f:
        for line in f:
            # Strip whitespace.
            line = line.strip()

            # Skip empty lines, comments, and section separators.
            if not line or line.startswith("#") or line == "---":
                continue

            # Skip constant definitions.
            if is_constant_definition(line):
                continue

            # Extract the type (first token on the line).
            parts = line.split()
            if not parts:
                continue

            type_str = parse_type(parts[0])

            # Check for external package reference (contains '/').
            if "/" in type_str:
                pkg_name, msg_name = type_str.split("/", 1)
                
                # Bug 1 fix: If pkg_name is the current package, treat it as internal.
                # Strip trailing "+" from current_pkg_name for comparison.
                if pkg_name == current_pkg_name.rstrip("+"):
                    if msg_name in interface_map:
                        target_dir = interface_map[msg_name]
                        if target_dir == rel_current_dir:
                            dep = f":{msg_name}"
                        else:
                            path = target_dir.replace(os.sep, "/")
                            dep = f"//{path}:{msg_name}"
                        deps.add(dep)
                    continue

                # Bug 2 fix: Consult global_interface_map for the correct subdirectory.
                # Default to "msg" if not found.
                target_sub_dir = "msg"
                if pkg_name in global_interface_map:
                    pkg_interface_map = global_interface_map[pkg_name]
                    if msg_name in pkg_interface_map:
                        target_sub_dir = pkg_interface_map[msg_name]
                        if not target_sub_dir:
                            # Root of the package
                            dep = f"@{pkg_name}//:{msg_name}"
                            deps.add(dep)
                            continue
                
                dep = f"@{pkg_name}//{target_sub_dir}:{msg_name}"
                deps.add(dep)
            elif type_str not in PRIMITIVE_TYPES:
                # Internal dependency - resolve the label based on its location
                if type_str in interface_map:
                    target_dir = interface_map[type_str]
                    if target_dir == rel_current_dir:
                        dep = f":{type_str}"
                    else:
                        # Use //path:Name format for different subdirectories
                        path = target_dir.replace(os.sep, "/")
                        dep = f"//{path}:{type_str}"
                    deps.add(dep)

    return sorted(deps)


def find_interface_files(directory):
    """Find all ROS 2 interface files in a directory.

    Scans the given directory (non-recursively) for files with interface
    extensions (.msg, .srv, .action, .idl).

    Args:
        directory: Path to the directory to scan.

    Returns:
        A sorted list of filenames (not full paths) that are interface files.
    """
    interface_files = []
    if not os.path.isdir(directory):
        return interface_files

    for entry in os.listdir(directory):
        filepath = os.path.join(directory, entry)
        if os.path.isfile(filepath):
            _, ext = os.path.splitext(entry)
            if ext in INTERFACE_EXTENSIONS:
                interface_files.append(entry)

    return sorted(interface_files)


def generate_build_target(filename, filepath, package_root, interface_map):
    """Generate a single Bazel rule string for an interface file.

    Args:
        filename: The interface file name (e.g., "Header.msg").
        filepath: Full path to the interface file.
        package_root: Root directory of the ROS package.
        interface_map: Dict mapping interface names to their relative directories.

    Returns:
        A string containing the Bazel rule definition.
    """
    return generate_build_target_v2(filename, filepath, package_root, interface_map, "", {})


def generate_build_target_v2(filename, filepath, package_root, interface_map, current_pkg_name, global_interface_map):
    """Generate a single Bazel rule string for an interface file.
    """
    name, ext = os.path.splitext(filename)
    rule_name = EXTENSION_TO_RULE[ext]
    deps = extract_dependencies_v2(filepath, package_root, interface_map, current_pkg_name, global_interface_map)

    lines = []
    lines.append(f"{rule_name}(")
    lines.append(f'    name = "{name}",')
    lines.append(f'    src = "{filename}",')

    if deps:
        lines.append("    deps = [")
        for dep in deps:
            lines.append(f'        "{dep}",')
        lines.append("    ],")

    lines.append(")")
    return "\n".join(lines)


def generate_build_file(interface_dir, package_root, interface_map):
    """Generate the complete BUILD.bazel content for a directory.

    Collects all interface files in the directory, determines which Bazel
    rules are needed, and writes the appropriate load statements followed
    by the rule definitions.

    Args:
        interface_dir: Path to the directory containing interface files.
        package_root: Root directory of the ROS package.
        interface_map: Dict mapping interface names to their relative directories.

    Returns:
        The complete BUILD.bazel file content as a string, or None if no
        interface files were found.
    """
    return generate_build_file_v2(interface_dir, package_root, interface_map, "", {}, "@rosidl_default_runtime//:defs.bzl")


def generate_build_file_v2(interface_dir, package_root, interface_map, current_pkg_name, global_interface_map, runtime_bzl):
    """Generate the complete BUILD.bazel content for a directory.
    """
    files = find_interface_files(interface_dir)
    if not files:
        return None

    # Determine which rules we need to load.
    needed_rules = set()
    for filename in files:
        _, ext = os.path.splitext(filename)
        needed_rules.add(EXTENSION_TO_RULE[ext])

    # Collect the distinct extensions present.
    extensions_present = sorted(set(
        os.path.splitext(f)[1] for f in files
    ))
    # Build glob patterns like "*.msg", "*.srv", etc.
    glob_patterns = ", ".join(f'"*{ext}"' for ext in extensions_present)

    # Build the file content.
    parts = []

    # Copyright header.
    parts.append(COPYRIGHT_HEADER)

    # Load statement - sort the rules for deterministic output.
    sorted_rules = sorted(needed_rules)
    rule_list = ", ".join(f'"{r}"' for r in sorted_rules)
    parts.append(
        f'load("{runtime_bzl}", {rule_list})\n'
    )

    # Package default visibility.
    parts.append('package(default_visibility = ["//visibility:public"])\n')

    # Export interface source files so other modules can depend on them.
    parts.append(f"exports_files(glob([{glob_patterns}]))\n")

    # Generate targets.
    targets = []
    for filename in files:
        filepath = os.path.join(interface_dir, filename)
        targets.append(generate_build_target_v2(
            filename, filepath, package_root, interface_map, current_pkg_name, global_interface_map))

    parts.append("\n\n".join(targets))
    parts.append("")  # Trailing newline.

    return "\n".join(parts)


def process_directory(base_dir, subdirs=None):
    """Process a directory tree, generating BUILD.bazel files for interfaces.

    Walks through subdirectories of the base directory. For each package
    directory, checks for msg/, srv/, action/ subdirectories containing
    interface files and generates BUILD.bazel files.

    Args:
        base_dir: The root directory to process.
        subdirs: Optional list of specific subdirectory names to process.
                 If None, all subdirectories are processed.

    Returns:
        A count of how many BUILD.bazel files were generated.
    """
    base_dir = os.path.abspath(base_dir)
    generated_count = 0

    if not os.path.isdir(base_dir):
        print(f"Error: '{base_dir}' is not a directory.", file=sys.stderr)
        return 0

    # Collect directories to process.
    if subdirs:
        # If specific subdirectories are given, use those exactly.
        dirs_to_process = []
        for sub in subdirs:
            subpath = os.path.join(base_dir, sub)
            if os.path.isdir(subpath):
                dirs_to_process.append(subpath)
            else:
                print(
                    f"Warning: subdirectory '{sub}' not found in "
                    f"'{base_dir}', skipping.",
                    file=sys.stderr,
                )
        dirs_to_process.sort()
    else:
        # Otherwise, list all subdirectories, applying skip rules.
        dirs_to_process = []
        for entry in sorted(os.listdir(base_dir)):
            # Skip directories starting with '_' or 'bazel-'.
            if entry.startswith("_") or entry.startswith("bazel-"):
                continue
            entry_path = os.path.join(base_dir, entry)
            if os.path.isdir(entry_path):
                dirs_to_process.append(entry_path)

    base_dir = os.path.abspath(base_dir)
    generated_count = 0

    # First pass: Build global interface map for all packages.
    global_interface_map = {}
    for package_dir in dirs_to_process:
        package_name = os.path.basename(package_dir)
        global_interface_map[package_name] = map_package_interfaces(package_dir)

    for package_dir in dirs_to_process:
        package_name = os.path.basename(package_dir)
        interface_map = global_interface_map[package_name]

        # Collect all unique directories containing interface files within the package.
        unique_dirs = sorted(set(
            d for d in interface_map.values() if "ros1" not in d
        ))

        for rel_dir in unique_dirs:
            # Skip the package root here; it's handled separately below.
            if not rel_dir:
                continue

            runtime_bzl = detect_rosidl_runtime(package_dir)

            interface_path = os.path.join(package_dir, rel_dir)
            
            content = generate_build_file_v2(
                interface_path, package_dir, interface_map, package_name, global_interface_map, runtime_bzl)
            if content is None:
                continue

            build_file = os.path.join(interface_path, "BUILD.bazel")
            with open(build_file, "w") as f:
                f.write(content)

            relative_path = os.path.relpath(build_file, base_dir)
            print(f"Generated: {relative_path}")
            generated_count += 1

        # Also check if the package directory itself contains interface files
        # (not in a msg/srv/action subdirectory).
        runtime_bzl = detect_rosidl_runtime(package_dir)
        content = generate_build_file_v2(package_dir, package_dir, interface_map, package_name, global_interface_map, runtime_bzl)
        if content is not None:
            build_file = os.path.join(package_dir, "BUILD.bazel")
            # Only write if there isn't already a BUILD.bazel for this dir
            # that we'd be overwriting (from the subdirectory processing).
            if not os.path.exists(build_file):
                with open(build_file, "w") as f:
                    f.write(content)

                relative_path = os.path.relpath(build_file, base_dir)
                print(f"Generated: {relative_path}")
                generated_count += 1

    return generated_count


def main():
    """Entry point for the interface_autogen tool."""
    parser = argparse.ArgumentParser(
        description=(
            "Generate BUILD.bazel files for ROS 2 interface definitions. "
            "Iterates through subdirectories of the specified directory, "
            "locating .msg, .srv, .action, and .idl files and generating "
            "the corresponding Bazel build rules."
        ),
    )
    parser.add_argument(
        "directory",
        help="Root directory to scan for interface files.",
    )
    parser.add_argument(
        "subdirectories",
        nargs="*",
        help=(
            "Optional list of specific subdirectories to process. "
            "If not specified, all subdirectories are processed "
            "(except those starting with '_' or 'bazel-')."
        ),
    )

    args = parser.parse_args()

    subdirs = args.subdirectories if args.subdirectories else None
    count = process_directory(args.directory, subdirs)

    print(f"\nGenerated {count} BUILD.bazel file(s).")


if __name__ == "__main__":
    main()
