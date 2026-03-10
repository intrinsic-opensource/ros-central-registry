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
Synchronizes and pushes updates across multiple git submodules.

For each submodule located within the 'submodules' folder relative to a given working directory,
this script can perform a clean, a commit, or a push.
"""

import os
import subprocess
from pathlib import Path
import argparse

def run_git_cmd(cmd, cwd, check=True):
    print(f"[{cwd.name}] Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)
    if check and result.returncode != 0:
        print(f"Error running {' '.join(cmd)} in {cwd}:")
        print(f"STDOUT: {result.stdout}")
        print(f"STDERR: {result.stderr}")
        result.check_returncode()
    return result

def main():
    parser = argparse.ArgumentParser(description="Synchronize submodules.")
    parser.add_argument("working_dir", help="Working directory to look for the 'submodules' folder relative to.")
    
    subparsers = parser.add_subparsers(dest="command", required=True, help="Submodule operation to perform")
    
    clean_parser = subparsers.add_parser("clean", help="Iterates over the submodules removing any staged or unstaged changes.")
    
    commit_parser = subparsers.add_parser("commit", help="Commit changes, and force check out the branch name to point to the resulting commit.")
    commit_parser.add_argument("branch_name", help="The branch name to update and check out.")
    
    push_parser = subparsers.add_parser("push", help="Push submodule changes. Also checkout the given branch, commit changes, and push to origin.")
    push_parser.add_argument("branch_name", nargs="?", default="rolling-bazel", help="The branch name to push to (defaults to rolling-bazel).")

    args = parser.parse_args()

    base_dir = Path(args.working_dir).resolve()
    submodules_dir = base_dir / "submodules"

    if not submodules_dir.exists() or not submodules_dir.is_dir():
        print(f"Error: '{submodules_dir}' does not exist or is not a directory.")
        return

    for submodule_path in submodules_dir.iterdir():
        if not submodule_path.is_dir():
            continue
            
        if not (submodule_path / ".git").exists():
            print(f"[{submodule_path.name}] Skipping because it doesn't look like a git repository.")
            continue

        print(f"\n{'='*50}")
        print(f"Processing submodule: {submodule_path.name}")
        print(f"{'='*50}")

        if args.command == "clean":
            run_git_cmd(["git", "reset", "--hard"], cwd=submodule_path)
            run_git_cmd(["git", "clean", "-fd"], cwd=submodule_path)
            
        elif args.command == "commit":
            # Commit changes and force checkout the branch name to point to the resulting commit
            run_git_cmd(["git", "add", "-A"], cwd=submodule_path)
            commit_status = run_git_cmd(["git", "status", "--porcelain"], cwd=submodule_path)
            if commit_status.stdout.strip():
                run_git_cmd(["git", "commit", "-m", "synchronizing package"], cwd=submodule_path)
            else:
                print(f"[{submodule_path.name}] No changes to commit.")
                
            run_git_cmd(["git", "checkout", "-B", args.branch_name], cwd=submodule_path)
            
        elif args.command == "push":
            # Ensure origin is set correctly
            remote_url = f"git@github.com:asymingt/{submodule_path.name}.git"
            remote_status = run_git_cmd(["git", "remote"], cwd=submodule_path, check=False)
            if "origin" in remote_status.stdout.split():
                run_git_cmd(["git", "remote", "set-url", "origin", remote_url], cwd=submodule_path)
            else:
                run_git_cmd(["git", "remote", "add", "origin", remote_url], cwd=submodule_path)

            # 1. Remove all 'MODULE.bazel.lock' files
            for lock_file in submodule_path.rglob("MODULE.bazel.lock"):
                print(f"[{submodule_path.name}] Removing {lock_file.relative_to(submodule_path)}")
                lock_file.unlink()

            run_git_cmd(["git", "checkout", "-B", args.branch_name], cwd=submodule_path)

            run_git_cmd(["git", "add", "-A"], cwd=submodule_path)
            
            commit_status = run_git_cmd(["git", "status", "--porcelain"], cwd=submodule_path)
            if commit_status.stdout.strip():
                run_git_cmd(["git", "commit", "-m", "synchronizing package"], cwd=submodule_path)
            else:
                print(f"[{submodule_path.name}] No changes to commit.")

            # 4. Push the changes
            run_git_cmd(["git", "push", "--force", "origin", "--set-upstream", args.branch_name], cwd=submodule_path)

if __name__ == "__main__":
    main()
