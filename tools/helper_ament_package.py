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

import os
import re

# Resolve vendor directory relative to this script
script_dir = os.path.dirname(os.path.abspath(__file__))
workspace_root = os.path.dirname(script_dir)
vendor_dir = os.path.join(workspace_root, "vendor")

def find_matching_paren(content, start_idx):
    count = 0
    for idx in range(start_idx, len(content)):
        char = content[idx]
        if char == '(':
            count += 1
        elif char == ')':
            count -= 1
            if count == 0:
                return idx
    return -1

def main():
    # 1. Get the list of all ROS modules in the vendor directory (excluding _vendor)
    if not os.path.exists(vendor_dir):
        print(f"Error: Vendor directory {vendor_dir} does not exist.")
        return

    vendor_subdirs = [d for d in os.listdir(vendor_dir) if os.path.isdir(os.path.join(vendor_dir, d))]
    ros_modules = {}
    for d in vendor_subdirs:
        if d.endswith('+'):
            mod_name = d[:-1]
        else:
            mod_name = d
        if mod_name.endswith('_vendor'):
            continue
        ros_modules[mod_name] = d

    # 2. Iterate through each vendor package and fix it
    for mod_name, dir_name in sorted(ros_modules.items()):
        pkg_dir = os.path.join(vendor_dir, dir_name)
        module_bazel_path = os.path.join(pkg_dir, "MODULE.bazel")
        build_bazel_path = os.path.join(pkg_dir, "BUILD.bazel")
        
        if not os.path.exists(module_bazel_path) or not os.path.exists(build_bazel_path):
            continue
            
        # Parse MODULE.bazel for bazel_dep
        with open(module_bazel_path, 'r') as f:
            module_content = f.read()
            
        bazel_deps_declared = set()
        for match in re.finditer(r'bazel_dep\s*\(\s*name\s*=\s*["\'](.*?)["\']', module_content):
            bazel_deps_declared.add(match.group(1))
            
        # Filter to only include dependencies that are present in the vendor directory
        vendor_bazel_deps = {dep for dep in bazel_deps_declared if dep in ros_modules}
        
        # Read BUILD.bazel
        with open(build_bazel_path, 'r') as f:
            build_content = f.read()
            
        # Find ament_package(...) rule call with matching parenthesis parsing
        start_idx = build_content.find("ament_package(")
        if start_idx == -1:
            continue
            
        open_paren_idx = build_content.find("(", start_idx)
        close_paren_idx = find_matching_paren(build_content, open_paren_idx)
        if close_paren_idx == -1:
            continue
            
        ament_full_match = build_content[start_idx : close_paren_idx + 1]
        ament_args = build_content[open_paren_idx + 1 : close_paren_idx]
        
        # Search for deps = [...] inside ament_args
        deps_match = re.search(r'deps\s*=\s*\[(.*?)\]', ament_args, re.DOTALL)
        
        if deps_match:
            deps_list_str = deps_match.group(1)
            # Parse existing dependencies
            existing_deps = []
            for line in deps_list_str.split('\n'):
                line_clean = line.strip()
                if line_clean:
                    existing_deps.append(line_clean)
                    
            # Find which vendor deps are already satisfied
            satisfied_vendor_deps = set()
            for dep in existing_deps:
                match = re.match(r'^["\']@([^/:]+)', dep)
                if match:
                    satisfied_vendor_deps.add(match.group(1))
                elif dep.startswith('":') or dep.startswith('":"'):
                    satisfied_vendor_deps.add(mod_name)
                    
            missing_deps = vendor_bazel_deps - satisfied_vendor_deps
            if not missing_deps:
                continue
                
            # Add missing deps to the list of existing deps
            new_deps = list(existing_deps)
            for dep in sorted(list(missing_deps)):
                new_deps.append(f'"@{dep}//:ament_package",')
                
            # Clean up and sort target strings
            dep_targets = []
            other_lines = []
            for d in new_deps:
                d_clean = d.strip().strip(',').strip('"').strip("'")
                if d_clean.startswith('@') or d_clean.startswith(':'):
                    dep_targets.append(d_clean)
                else:
                    other_lines.append(d)
                    
            # Format the new sorted target list
            sorted_formatted_deps = []
            for d in sorted(dep_targets):
                sorted_formatted_deps.append(f'        "{d}",')
            for d in other_lines:
                sorted_formatted_deps.append(f'        {d}')
                
            new_deps_str = '\n' + '\n'.join(sorted_formatted_deps) + '\n    '
            
            # Replace the deps list inside ament_args
            new_ament_args = ament_args.replace(deps_list_str, new_deps_str)
            new_ament_call = ament_full_match.replace(ament_args, new_ament_args)
            new_build_content = build_content.replace(ament_full_match, new_ament_call)
            
        else:
            # deps = [...] is not present at all. Let's add it.
            # Format deps list
            formatted_deps = []
            for dep in sorted(list(vendor_bazel_deps)):
                formatted_deps.append(f'        "@{dep}//:ament_package",')
            
            # Clean up ending parenthesis/whitespace
            last_paren_idx = ament_full_match.rfind(')')
            args_stripped = ament_full_match[:last_paren_idx].rstrip()
            
            if args_stripped.endswith(','):
                deps_str = '\n    deps = [\n' + '\n'.join(formatted_deps) + '\n    ]'
            else:
                deps_str = ',\n    deps = [\n' + '\n'.join(formatted_deps) + '\n    ]'
                
            new_ament_call = args_stripped + deps_str + '\n)'
            new_build_content = build_content.replace(ament_full_match, new_ament_call)
            
        # Write the modified content back
        with open(build_bazel_path, 'w') as f:
            f.write(new_build_content)
            
        print(f"Fixed dependencies for package {mod_name}")

if __name__ == "__main__":
    main()
