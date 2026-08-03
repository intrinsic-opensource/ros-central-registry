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
Safely re-baseline a vendored, locally-edited module against whatever is
currently on disk under modules/ (e.g. after a `git pull`/rebase brought in
a concurrent edit to a not-yet-published version), while preserving the
developer's own uncommitted edits.

//tools/ci:create_patch refuses (rather than guessing) when it detects the
on-disk modules/<module>/<version>/ directory has drifted since this
workspace vendored it -- this tool is the recovery path from that refusal,
replacing the fully-manual "re-run setup-workspace, re-vendor, redo your
edits by hand" dance (which discards uncommitted work) with one command.
"""

import argparse
import dataclasses
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from tools.ci import create_patch
from tools.ci import vendor_modules
from tools.ci.vendor_modules import parse_module_tokens


def get_current_release(target_workspace: Path) -> str:
    """
    Reads the `ros` bazel_dep version out of workspace/MODULE.bazel (as
    written by //tools/ci:setup_workspace), so a rebase can re-run
    setup_workspace against the same release without the developer having
    to re-specify --release.
    """
    module_dot_bazel = target_workspace / "MODULE.bazel"
    content = module_dot_bazel.read_text()
    match = re.search(r'bazel_dep\(\s*name\s*=\s*"ros"\s*,\s*version\s*=\s*"([^"]+)"', content)
    if not match:
        raise RuntimeError(
            f"Could not find bazel_dep(name=\"ros\", ...) in {module_dot_bazel} "
            "-- pass --release explicitly."
        )
    return match.group(1)


def capture_edit_diff(
    snapshot_dir: Path, vendor_module_dir: Path
) -> Tuple[Dict[str, str], Dict[str, str], Optional[str]]:
    """
    Computes a developer's own edits relative to the last vendor snapshot
    (NOT raw upstream -- see create_patch.diff_module_source, which this
    reuses for everything except MODULE.bazel, which it deliberately never
    diffs). Returns (patches, overlays, new_module_bazel_content_or_None).
    """
    patches, overlays = create_patch.diff_module_source(snapshot_dir, vendor_module_dir)

    old_module_bazel = snapshot_dir / "MODULE.bazel"
    new_module_bazel = vendor_module_dir / "MODULE.bazel"
    module_bazel_diff = None
    old_content = old_module_bazel.read_text() if old_module_bazel.exists() else ""
    new_content = new_module_bazel.read_text() if new_module_bazel.exists() else ""
    if old_content != new_content:
        module_bazel_diff = new_content

    return patches, overlays, module_bazel_diff


def apply_patch_text(patch_text: str, target_dir: Path) -> bool:
    """
    Applies a unified diff (as produced by create_patch.diff_module_source)
    against target_dir via the system `patch` CLI, which resolves the
    target file itself from the diff's own "a/<path>"/"b/<path>" headers.
    Returns True on a clean apply; on conflict, `patch` writes the usual
    <file>.rej next to the target and this returns False without raising,
    so one conflicting file doesn't abort the rest.
    """
    result = subprocess.run(
        ["patch", "-p1", "--forward", "--no-backup-if-mismatch"],
        input=patch_text,
        capture_output=True,
        text=True,
        cwd=target_dir,
    )
    return result.returncode == 0


def reapply_edit_diff(
    patches: Dict[str, str], overlays: Dict[str, str], target_dir: Path
) -> Tuple[List[str], List[str]]:
    """
    Reapplies a captured edit-diff onto a freshly-vendored tree. Patches
    are applied via the system `patch` CLI; overlays (brand-new files) are
    just written directly, since a full-file write always "applies"
    regardless of the fresh baseline's content -- but if the fresh
    baseline already has different content at that same path, that's
    reported as a conflict too (the developer's content still wins, since
    it's what they explicitly asked to keep, but they should look).
    Returns (conflicting_patch_names, conflicting_overlay_paths).
    """
    conflicting_patches = []
    for patch_name, patch_text in sorted(patches.items()):
        if not apply_patch_text(patch_text, target_dir):
            conflicting_patches.append(patch_name)

    conflicting_overlays = []
    for rel_path, content in sorted(overlays.items()):
        overlay_path = target_dir / rel_path
        if overlay_path.exists() and overlay_path.read_text() != content:
            conflicting_overlays.append(rel_path)
        overlay_path.parent.mkdir(parents=True, exist_ok=True)
        overlay_path.write_text(content)

    return conflicting_patches, conflicting_overlays


@dataclasses.dataclass
class RebaseResult:
    """Summary of one module's rebase, for main() to report to the developer."""
    module: str
    applied_patches: List[str]
    conflicting_patches: List[str]
    applied_overlays: List[str]
    conflicting_overlays: List[str]
    module_bazel_needs_manual_reconciliation: bool
    module_bazel_your_edits_path: Optional[Path]


def rebase_module(
    module: str,
    repo_root: Path,
    target_workspace: Path,
    release: Optional[str] = None,
    dry_run: bool = False,
) -> RebaseResult:
    """
    Preserves a developer's edits to `module`, re-baselines the workspace
    against whatever is currently on disk under modules/ (a fresh
    setup_workspace + vendor_modules run), then reapplies those edits on
    top. MODULE.bazel is a special case: for rosdistro (the only module
    ever expected to have real MODULE.bazel edits), the developer's edited
    content is written aside for manual reconciliation rather than
    auto-applied -- see the module-level docstring on why an automatic
    Starlark merge isn't attempted.
    """
    vendor_module_dir = target_workspace / "vendor" / f"{module}+"
    if not vendor_module_dir.exists():
        raise RuntimeError(
            f"{vendor_module_dir} does not exist. Run "
            f"'bazel run //tools/ci:vendor_modules -- {module}' first."
        )
    snapshot_dir = target_workspace / vendor_modules.VENDOR_SNAPSHOT_DIR_NAME / f"{module}+"
    if not snapshot_dir.exists():
        raise RuntimeError(
            f"{snapshot_dir} does not exist -- this workspace was vendored "
            "before rebase-module support was added, or the snapshot was "
            f"otherwise lost. Re-run 'bazel run //tools/ci:vendor_modules -- "
            f"{module}' (this discards any uncommitted edits) before "
            "rebasing."
        )

    patches, overlays, new_module_bazel = capture_edit_diff(snapshot_dir, vendor_module_dir)

    if dry_run:
        print(f"Would rebase {module}: {len(patches)} patch(es), {len(overlays)} overlay(s), "
              f"MODULE.bazel edited: {new_module_bazel is not None}.")
        return RebaseResult(
            module=module, applied_patches=[], conflicting_patches=[],
            applied_overlays=[], conflicting_overlays=[],
            module_bazel_needs_manual_reconciliation=False,
            module_bazel_your_edits_path=None,
        )

    release = release or get_current_release(target_workspace)

    print(f"Re-baselining workspace against release {release}...")
    subprocess.run(
        ["bazel", "run", "//tools/ci:setup_workspace", "--",
         f"--release={release}", f"--workspace-dir={target_workspace}"],
        cwd=repo_root, check=True,
    )
    print(f"Re-vendoring {module}...")
    subprocess.run(
        ["bazel", "run", "//tools/ci:vendor_modules", "--",
         module, f"--workspace-dir={target_workspace}"],
        cwd=repo_root, check=True,
    )

    conflicting_patches, conflicting_overlays = reapply_edit_diff(
        patches, overlays, vendor_module_dir
    )
    applied_patches = [p for p in patches if p not in conflicting_patches]
    applied_overlays = [o for o in overlays if o not in conflicting_overlays]

    module_bazel_needs_manual_reconciliation = False
    module_bazel_your_edits_path = None
    if new_module_bazel is not None:
        if module == "rosdistro":
            conflicts_dir = target_workspace / ".rebase_conflicts"
            conflicts_dir.mkdir(parents=True, exist_ok=True)
            module_bazel_your_edits_path = conflicts_dir / f"{module}.MODULE.bazel.your-edits"
            module_bazel_your_edits_path.write_text(new_module_bazel)
            module_bazel_needs_manual_reconciliation = True
        else:
            print(
                f"Note: {module}'s vendored MODULE.bazel was edited, but "
                "MODULE.bazel edits are never preserved for non-rosdistro "
                "modules (see AGENTS.md) -- dropped."
            )

    return RebaseResult(
        module=module,
        applied_patches=applied_patches,
        conflicting_patches=conflicting_patches,
        applied_overlays=applied_overlays,
        conflicting_overlays=conflicting_overlays,
        module_bazel_needs_manual_reconciliation=module_bazel_needs_manual_reconciliation,
        module_bazel_your_edits_path=module_bazel_your_edits_path,
    )


def print_rebase_result(result: RebaseResult) -> None:
    print(f"{result.module}:")
    print(f"  patches applied cleanly: {len(result.applied_patches)}")
    if result.conflicting_patches:
        print(f"  patches with CONFLICTS (.rej files written): {', '.join(result.conflicting_patches)}")
    print(f"  overlays written: {len(result.applied_overlays)}")
    if result.conflicting_overlays:
        print(f"  overlays that collided with fresh baseline content (your version kept): "
              f"{', '.join(result.conflicting_overlays)}")
    if result.module_bazel_needs_manual_reconciliation:
        print(
            "  MODULE.bazel: your edits were NOT auto-applied -- reconcile them by hand "
            f"from {result.module_bazel_your_edits_path} into the freshly-vendored "
            "MODULE.bazel."
        )


def main():
    parser = argparse.ArgumentParser(
        description="Preserve a vendored module's local edits while "
        "re-baselining it against the current on-disk modules/ state -- "
        "the recovery path from //tools/ci:create_patch's drift-detection "
        "refusal."
    )
    parser.add_argument(
        "modules",
        nargs="+",
        help="Name(s) of the module(s) to rebase, e.g. rosdistro. A single "
        'space- or comma-separated string is also accepted, e.g. '
        '"rclcpp rclcpp_action".',
    )
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=Path("workspace"),
        help="Workspace directory created by //tools/ci:setup_workspace",
    )
    parser.add_argument(
        "--release",
        default=None,
        help="Release to re-baseline against. Defaults to whatever the "
        "workspace's own MODULE.bazel already pins.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be rebased, but don't touch the workspace.",
    )
    args = parser.parse_args()

    repo_root = Path(os.environ.get("BUILD_WORKSPACE_DIRECTORY", ".")).resolve()
    target_workspace = (repo_root / args.workspace_dir).resolve()

    if not (target_workspace / "MODULE.bazel").exists():
        print(
            f"Error: {target_workspace / 'MODULE.bazel'} does not exist. "
            "Run //tools/ci:setup_workspace first.",
            file=sys.stderr,
        )
        sys.exit(1)

    modules = parse_module_tokens(args.modules)
    try:
        results = [
            rebase_module(module, repo_root, target_workspace, args.release, args.dry_run)
            for module in modules
        ]
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    if args.dry_run:
        return

    for result in results:
        print_rebase_result(result)
    print(
        "Done. Resolve any conflicts above (search for .rej files under "
        "workspace/vendor/, and reconcile MODULE.bazel by hand if noted), "
        "then re-test and re-run //tools/ci:create_patch."
    )


if __name__ == "__main__":
    main()
