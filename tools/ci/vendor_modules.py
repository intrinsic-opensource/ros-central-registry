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
Vendor a set of modules into a Bazel workspace created by
//tools/ci:setup_workspace, so their sources can be edited in place.
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

from tools.ci.bzlmod_lib import scan_module_for_dependencies

VENDOR_DIR_RC_LINE = "common --vendor_dir=vendor"


def declared_module_names(module_dot_bazel: Path, modules_dir: Path) -> set:
    return set(
        scan_module_for_dependencies(
            module_dot_bazel, modules_dir, include_bcr=True, include_rcr=True
        ).keys()
    )


def ensure_vendor_dir_flag(bazelrc: Path) -> None:
    """
    Older workspaces may have been created before setup_workspace started
    writing --vendor_dir into the .bazelrc. Patch it in so builds/tests in
    this workspace actually read from the vendored sources.
    """
    content = bazelrc.read_text()
    if VENDOR_DIR_RC_LINE in content:
        return
    with open(bazelrc, "a") as f:
        f.write(f"\n{VENDOR_DIR_RC_LINE}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Vendor a set of modules into a Bazel workspace for "
        "in-place editing, e.g. to test local patches."
    )
    parser.add_argument(
        "modules",
        nargs="+",
        help="Names of modules to vendor, e.g. rclcpp rmw_fastrtps_cpp",
    )
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=Path("workspace"),
        help="Workspace directory created by //tools/ci:setup_workspace",
    )
    args = parser.parse_args()

    workspace_root = Path(
        os.environ.get("BUILD_WORKSPACE_DIRECTORY", ".")
    ).resolve()
    target_workspace = (workspace_root / args.workspace_dir).resolve()
    module_dot_bazel = target_workspace / "MODULE.bazel"
    bazelrc = target_workspace / ".bazelrc"

    if not module_dot_bazel.exists():
        print(
            f"Error: {module_dot_bazel} does not exist. Run "
            "//tools/ci:setup_workspace first.",
            file=sys.stderr,
        )
        sys.exit(1)

    declared = declared_module_names(module_dot_bazel, workspace_root / "modules")
    unknown = [m for m in args.modules if m not in declared]
    if unknown:
        print(
            f"Error: {', '.join(unknown)} not declared as bazel_dep in "
            f"{module_dot_bazel}.",
            file=sys.stderr,
        )
        sys.exit(1)

    ensure_vendor_dir_flag(bazelrc)

    cmd = ["bazel", "vendor", "--vendor_dir=vendor"]
    cmd += [f"--repo=@{m}" for m in args.modules]
    print(f"Vendoring {', '.join(args.modules)} into "
          f"{target_workspace / 'vendor'}...")
    subprocess.run(cmd, cwd=target_workspace, check=True)

    print(
        "Done. Edit sources under "
        f"{target_workspace / 'vendor'} to test local patches; "
        "builds/tests in this workspace will pick them up automatically."
    )


if __name__ == "__main__":
    main()
