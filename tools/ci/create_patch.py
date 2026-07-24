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
Roll a developer's in-place edits to a single vendored module (see
//tools/ci:vendor_modules) up into a brand-new module version, e.g.
rclcpp@32.0.0-1.rcr.1 -> rclcpp@32.0.0-1.rcr.2.
"""

import argparse
import base64
import difflib
import filecmp
import hashlib
import json
import os
import re
import shutil
import sys
import tarfile
import tempfile
import urllib.request
import zipfile
from pathlib import Path
from typing import Dict, Tuple

from tools.ci import bzlmod_lib


def read_module_version(module_dot_bazel: Path, package_name: str) -> str:
    """
    Read the version a MODULE.bazel's own module(name=package_name, ...)
    declaration was resolved to. Used instead of scanning bazel_dep text in
    the root workspace, since bazel_dep text doesn't reflect
    single_version_override()s -- the vendored tree's own MODULE.bazel is
    the source of truth for what Bazel actually resolved.
    """
    content = module_dot_bazel.read_text()
    match = re.search(
        r'module\(\s*name\s*=\s*"' + re.escape(package_name) + r'"\s*,\s*version\s*=\s*"([^"]+)"',
        content,
    )
    if not match:
        raise RuntimeError(
            f"Could not find module(name={package_name!r}, ...) in {module_dot_bazel}"
        )
    return match.group(1)


def load_existing_patches_and_overlays(module_dir: Path) -> Tuple[Dict[str, str], Dict[str, str]]:
    """
    Load the currently-published patches/overlay content for a module
    version, keyed the same way diff_module_source() keys its output, so
    the two can be compared for the no-op case.
    """
    with open(module_dir / "source.json", "r") as f:
        source = json.load(f)
    result = {"patches": {}, "overlay": {}}
    for key in ("patches", "overlay"):
        for rel_path in source.get(key, {}).keys():
            with open(module_dir / key / rel_path, "r") as f:
                result[key][rel_path] = f.read()
    return result["patches"], result["overlay"]


def fetch_raw_upstream(source_json_path: Path, dest_dir: Path) -> None:
    """
    Fetch and extract a module's raw upstream archive (no patches/overlay
    applied) into dest_dir, verifying integrity against source.json.

    This matters because a module version's patches/overlay always
    describe a transformation relative to RAW upstream, not relative to a
    previously-patched version -- every .rcr.N of a package shares the
    same url/integrity/strip_prefix (carried forward by this script's own
    copytree step), so diffing a developer's edits against an
    already-patched reference would silently compute patches that don't
    apply cleanly against upstream once published.
    """
    with open(source_json_path, "r") as f:
        source = json.load(f)
    url = source["url"]
    strip_prefix = source.get("strip_prefix", "")
    expected_integrity = source["integrity"]

    with urllib.request.urlopen(url) as response:
        data = response.read()

    algorithm, expected_digest = expected_integrity.split("-", 1)
    if algorithm != "sha256":
        raise RuntimeError(f"Unsupported integrity algorithm in {source_json_path}: {algorithm}")
    actual_digest = base64.b64encode(hashlib.sha256(data).digest()).decode()
    if actual_digest != expected_digest:
        raise RuntimeError(
            f"Integrity check failed fetching {url}: expected {expected_integrity}, "
            f"got sha256-{actual_digest}"
        )

    dest_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as raw_extract_dir_str:
        raw_extract_dir = Path(raw_extract_dir_str)
        archive_path = raw_extract_dir / "archive"
        archive_path.write_bytes(data)
        if url.endswith(".zip"):
            with zipfile.ZipFile(archive_path) as zf:
                zf.extractall(raw_extract_dir)
        else:
            with tarfile.open(archive_path, mode="r:*") as tf:
                tf.extractall(raw_extract_dir, filter="data")
        archive_path.unlink()
        extracted_root = raw_extract_dir / strip_prefix if strip_prefix else raw_extract_dir
        for item in extracted_root.iterdir():
            shutil.move(str(item), str(dest_dir / item.name))


def diff_module_source(pristine_dir: Path, edited_dir: Path) -> Tuple[Dict[str, str], Dict[str, str]]:
    """
    Diff a pristine (unedited) vendored module tree against a developer's
    edited copy. Changed files become unified-diff patches; brand-new
    files become overlays; files deleted by the developer become deletion
    patches. MODULE.bazel is never diffed -- it's authored separately.
    """
    patches: Dict[str, str] = {}
    overlays: Dict[str, str] = {}

    for edited_file in sorted(edited_dir.rglob("*")):
        if not edited_file.is_file() or edited_file.name == "MODULE.bazel":
            continue
        rel_path = edited_file.relative_to(edited_dir)
        pristine_file = pristine_dir / rel_path
        if pristine_file.exists():
            if filecmp.cmp(pristine_file, edited_file, shallow=False):
                continue
            with open(pristine_file, "r") as f_ref, open(edited_file, "r") as f_new:
                diff_lines = difflib.unified_diff(
                    f_ref.readlines(), f_new.readlines(),
                    fromfile=f"a/{rel_path}", tofile=f"b/{rel_path}",
                )
                patches[bzlmod_lib.patch_file_name_from_rel_path(rel_path)] = "".join(diff_lines)
        else:
            with open(edited_file, "r") as f:
                overlays[str(rel_path)] = f.read()

    for pristine_file in sorted(pristine_dir.rglob("*")):
        if not pristine_file.is_file() or pristine_file.name == "MODULE.bazel":
            continue
        rel_path = pristine_file.relative_to(pristine_dir)
        if not (edited_dir / rel_path).exists():
            with open(pristine_file, "r") as f_ref:
                diff_lines = difflib.unified_diff(
                    f_ref.readlines(), [],
                    fromfile=f"a/{rel_path}", tofile="/dev/null",
                )
                patches[bzlmod_lib.patch_file_name_from_rel_path(rel_path)] = "".join(diff_lines)

    return patches, overlays


def main():
    parser = argparse.ArgumentParser(
        description="Create a new patch version of a single vendored module "
        "from in-place edits made via //tools/ci:vendor_modules."
    )
    parser.add_argument("module", help="Name of the module to create a patch for, e.g. rclcpp")
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=Path("workspace"),
        help="Workspace directory created by //tools/ci:setup_workspace",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and print what would happen, but don't write anything",
    )
    args = parser.parse_args()

    workspace_root = Path(os.environ.get("BUILD_WORKSPACE_DIRECTORY", ".")).resolve()
    modules_dir = workspace_root / "modules"
    target_workspace = (workspace_root / args.workspace_dir).resolve()

    module_dot_bazel = target_workspace / "MODULE.bazel"
    if not module_dot_bazel.exists():
        print(
            f"Error: {module_dot_bazel} does not exist. Run "
            "//tools/ci:setup_workspace first.",
            file=sys.stderr,
        )
        sys.exit(1)

    vendor_module_dir = target_workspace / "vendor" / f"{args.module}+"
    if not vendor_module_dir.exists():
        print(
            f"Error: {vendor_module_dir} does not exist. Run "
            f"'bazel run //tools/ci:vendor_modules -- {args.module}' first.",
            file=sys.stderr,
        )
        sys.exit(1)

    # Determine the version Bazel actually resolved for this module (which
    # reflects any single_version_override from setup_workspace), not the
    # textual bazel_dep pin in the root workspace's MODULE.bazel.
    current_version = read_module_version(vendor_module_dir / "MODULE.bazel", args.module)

    metadata_path = modules_dir / args.module / "metadata.json"
    metadata = bzlmod_lib.read_metadata_json(metadata_path)
    latest_published = bzlmod_lib.get_latest_non_yanked_version(metadata)
    if bzlmod_lib.version_sort_key(current_version) != bzlmod_lib.version_sort_key(latest_published):
        print(
            f"Error: this workspace resolved {args.module} to {current_version}, "
            f"but the latest published version is {latest_published}. This "
            "workspace is stale (likely set up before a concurrent patch "
            "landed). Re-run //tools/ci:setup_workspace, then "
            "//tools/ci:vendor_modules, before creating a patch.",
            file=sys.stderr,
        )
        sys.exit(1)

    current_module_dir = modules_dir / args.module / current_version

    # Obtain the raw (unpatched) upstream source for diffing -- NOT the
    # currently-published version's vendored tree, since that already has
    # existing patches/overlay applied (see fetch_raw_upstream's docstring
    # for why that distinction matters).
    pristine_root = target_workspace / ".pristine_upstream"
    try:
        print(f"Fetching raw upstream source for {args.module} (no patches applied)...")
        fetch_raw_upstream(current_module_dir / "source.json", pristine_root)
        new_patches, new_overlays = diff_module_source(pristine_root, vendor_module_dir)
    except Exception:
        print(
            f"Error occurred while computing the diff; leaving raw upstream "
            f"reference at {pristine_root} for inspection.",
            file=sys.stderr,
        )
        raise
    else:
        shutil.rmtree(pristine_root, ignore_errors=True)

    old_patches, old_overlays = load_existing_patches_and_overlays(current_module_dir)
    if new_patches == old_patches and new_overlays == old_overlays:
        print(f"No changes detected for {args.module}; nothing to do.")
        return

    new_version = bzlmod_lib.increment_version(current_version)
    new_module_dir = modules_dir / args.module / new_version

    print(f"Creating {args.module}@{new_version} (from {current_version}):")
    print(f"  patches: {len(new_patches)} (was {len(old_patches)})")
    print(f"  overlay: {len(new_overlays)} (was {len(old_overlays)})")

    if args.dry_run:
        print("Dry run: no files written.")
        return

    shutil.copytree(current_module_dir, new_module_dir, dirs_exist_ok=True)
    shutil.rmtree(new_module_dir / "patches", ignore_errors=True)
    shutil.rmtree(new_module_dir / "overlay", ignore_errors=True)
    for patch_name, content in new_patches.items():
        patch_path = new_module_dir / "patches" / patch_name
        patch_path.parent.mkdir(parents=True, exist_ok=True)
        patch_path.write_text(content)
    for overlay_name, content in new_overlays.items():
        overlay_path = new_module_dir / "overlay" / overlay_name
        overlay_path.parent.mkdir(parents=True, exist_ok=True)
        overlay_path.write_text(content)

    module_file = new_module_dir / "MODULE.bazel"
    module_file.write_text(
        bzlmod_lib.rewrite_module_version(module_file.read_text(), args.module, new_version)
    )
    bzlmod_lib.regenerate_integrity_hashes(new_module_dir)
    bzlmod_lib.add_version_to_metadata_json(metadata_path, new_version)

    print(f"Done. modules/{args.module}/{new_version}/ is ready to commit as a PR.")


if __name__ == "__main__":
    main()
