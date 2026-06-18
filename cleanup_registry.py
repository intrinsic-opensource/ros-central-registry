#!/usr/bin/env python3
import os
import shutil
import json
import ast

REGISTRY_DIR = os.path.dirname(os.path.abspath(__file__))
MODULES_DIR = os.path.join(REGISTRY_DIR, "modules")

def extract_value(node):
    if isinstance(node, ast.Constant):
        return node.value
    elif isinstance(node, ast.Str):
        return node.s
    return None

def get_deps(module_name, version):
    module_ver_dir = os.path.join(MODULES_DIR, module_name, version)
    module_file = os.path.join(module_ver_dir, "MODULE.bazel")
    if not os.path.exists(module_file):
        return []
    
    with open(module_file, 'r') as f:
        content = f.read()
    
    try:
        tree = ast.parse(content)
    except Exception as e:
        print(f"Error parsing {module_file}: {e}")
        return []
        
    deps = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'bazel_dep':
            name = None
            version = None
            if len(node.args) >= 1:
                name = extract_value(node.args[0])
            if len(node.args) >= 2:
                version = extract_value(node.args[1])
            for kw in node.keywords:
                if kw.arg == 'name':
                    name = extract_value(kw.value)
                elif kw.arg == 'version':
                    version = extract_value(kw.value)
            if name and version:
                deps.append((name, version))
    return deps

def get_transitive_closure(start_nodes):
    visited = set()
    queue = list(start_nodes)
    while queue:
        node = queue.pop(0)
        if node in visited:
            continue
        visited.add(node)
        
        name, version = node
        deps = get_deps(name, version)
        for dep in deps:
            if dep not in visited:
                queue.append(dep)
    return visited

def main():
    lyrical_starts = [
        ("ros", "lyrical.2026-06-08"),
        ("ros", "lyrical.2026-06-08.rcr.0"),
        ("ros", "lyrical.2026-06-08.rcr.1"),
    ]
    rolling_starts = [
        ("ros", "rolling.2026-04-15"),
        ("ros", "rolling.2026-04-15.rcr.0"),
        ("ros", "rolling.2026-04-15.rcr.1"),
    ]
    
    print("Calculating Lyrical transitive closure...")
    lyrical_closure = get_transitive_closure(lyrical_starts)
    
    print("Calculating Rolling transitive closure...")
    rolling_closure = get_transitive_closure(rolling_starts)
    
    to_remove = rolling_closure - lyrical_closure
    
    # Filter out 0.0.0 versions and modules/versions that don't exist locally
    final_to_remove = set()
    for name, version in to_remove:
        if version == "0.0.0":
            continue
        # Check if it exists locally in modules
        module_ver_dir = os.path.join(MODULES_DIR, name, version)
        if os.path.exists(module_ver_dir):
            final_to_remove.add((name, version))
            
    if not final_to_remove:
        print("No versions to remove.")
        return

    print(f"Removing {len(final_to_remove)} local module versions...")
    
    # Group by module name to update metadata.json once per module
    by_module = {}
    for name, version in final_to_remove:
        by_module.setdefault(name, []).append(version)
        
    for name, versions in sorted(by_module.items()):
        # 1. Delete version directories
        for version in versions:
            ver_dir = os.path.join(MODULES_DIR, name, version)
            print(f"Deleting {ver_dir}...")
            shutil.rmtree(ver_dir)
            
        # 2. Update metadata.json
        metadata_file = os.path.join(MODULES_DIR, name, "metadata.json")
        if os.path.exists(metadata_file):
            print(f"Updating {metadata_file}...")
            with open(metadata_file, 'r') as f:
                try:
                    data = json.load(f)
                except Exception as e:
                    print(f"Error loading {metadata_file}: {e}")
                    continue
                    
            if 'versions' in data:
                data['versions'] = [v for v in data['versions'] if v not in versions]
                has_0_0_0 = os.path.exists(os.path.join(MODULES_DIR, name, "0.0.0"))
                if has_0_0_0 and "0.0.0" not in data['versions']:
                    data['versions'].insert(0, "0.0.0")
            if 'yanked_versions' in data:
                data['yanked_versions'] = {v: reason for v, reason in data['yanked_versions'].items() if v not in versions}
                
            with open(metadata_file, 'w') as f:
                json.dump(data, f, indent=4)
                f.write('\n') # keep trailing newline
                
    print("Registry cleanup completed successfully.")

if __name__ == "__main__":
    main()
