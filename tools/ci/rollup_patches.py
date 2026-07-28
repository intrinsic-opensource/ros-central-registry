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
Weekly patch rollup: aggregate every package that has a newer published
version than what the current "ros" release references (leaf bumps
already created by //tools/ci:create_patch PRs during the week),
transitively bump every package that depends on one of them (so
bazel_dep pins stay consistent), regenerate the top-level "ros" module,
and bump the hardcoded CI matrix release string in the three
_check_distribution_*.yml workflows so the existing CI matrix actually
exercises the new candidate.
"""

import argparse
import os
import shutil
import sys
from pathlib import Path
from typing import Dict

from tools.ci import bzlmod_lib

WORKFLOW_FILES_WITH_HARDCODED_ROS_MATRIX = [
    ".github/workflows/_check_distribution_compiles.yml",
    ".github/workflows/_check_distribution_examples.yml",
    ".github/workflows/_check_distribution_tests.yml",
]


def find_current_ros_version(modules_dir: Path, distro: str, date: str) -> str:
    """
    Find the highest-numbered .rcr.N release for the given <distro>.<date>
    row under modules/ros/.
    """
    candidates = []
    ros_dir = modules_dir / "ros"
    for entry in ros_dir.iterdir():
        if not entry.is_dir():
            continue
        parts = entry.name.split(".")
        if len(parts) != 4 or parts[0] != distro or parts[1] != date or parts[2] != "rcr":
            continue
        if not parts[3].isdigit():
            continue
        candidates.append((int(parts[3]), entry.name))
    if not candidates:
        raise RuntimeError(
            f"No .rcr.N release found for {distro}.{date} under {ros_dir}; "
            "a rollup requires at least one existing patch release to build on."
        )
    return max(candidates)[1]


def check_no_yanked_dependencies(current_packages: Dict[str, str], modules_dir: Path) -> None:
    """
    Refuse to roll up on top of a currently-referenced package version
    that's already yanked -- that's a pre-existing data-integrity problem
    this rollup didn't cause and shouldn't silently build on top of.
    """
    for name, pinned_version in current_packages.items():
        metadata = bzlmod_lib.read_metadata_json(modules_dir / name / "metadata.json")
        yanked = metadata.get("yanked_versions", {})
        if pinned_version in yanked:
            raise RuntimeError(
                f"{name}@{pinned_version} is referenced by the current ros release "
                f"but is yanked ({yanked[pinned_version]}); refusing to roll up on "
                "top of a yanked dependency."
            )


def bump_ci_matrix_files(repo_root: Path, old_ros_version: str, new_ros_version: str, dry_run: bool) -> None:
    for rel_path in WORKFLOW_FILES_WITH_HARDCODED_ROS_MATRIX:
        path = repo_root / rel_path
        content = path.read_text()
        old_entry = f"- {old_ros_version}"
        new_entry = f"- {new_ros_version}"
        if old_entry not in content:
            raise RuntimeError(
                f"Expected to find {old_entry!r} in {rel_path}, found none; "
                "refusing to silently skip bumping the CI matrix."
            )
        print(f"  {rel_path}: {old_ros_version} -> {new_ros_version}")
        if not dry_run:
            path.write_text(content.replace(old_entry, new_entry, 1))


def main():
    parser = argparse.ArgumentParser(
        description="Aggregate this week's per-package patches into a new ros release."
    )
    parser.add_argument(
        "--release",
        required=True,
        help="<distribution>.<date> row to roll up, e.g. lyrical.2026-06-08",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and print what would happen, but don't write anything",
    )
    args = parser.parse_args()

    release_parts = args.release.split(".")
    if len(release_parts) != 2:
        print(f"Error: --release must be <distribution>.<date>, got {args.release!r}", file=sys.stderr)
        sys.exit(1)
    distro, date = release_parts

    repo_root = Path(os.environ.get("BUILD_WORKSPACE_DIRECTORY", ".")).resolve()
    modules_dir = repo_root / "modules"

    current_ros_version = find_current_ros_version(modules_dir, distro, date)
    print(f"Current ros release: {current_ros_version}")

    current_packages = bzlmod_lib.scan_module_for_dependencies(
        modules_dir / "ros" / current_ros_version / "MODULE.bazel", modules_dir
    )
    print(f"Found {len(current_packages)} packages referenced by {current_ros_version}.")

    check_no_yanked_dependencies(current_packages, modules_dir)

    latest: Dict[str, str] = {
        name: bzlmod_lib.get_latest_non_yanked_version(
            bzlmod_lib.read_metadata_json(modules_dir / name / "metadata.json")
        )
        for name in current_packages
    }

    pending_leaf_bumps = {
        name: latest[name]
        for name in current_packages
        if bzlmod_lib.version_sort_key(latest[name]) != bzlmod_lib.version_sort_key(current_packages[name])
    }

    if not pending_leaf_bumps:
        print(
            f"Nothing to roll up -- every package referenced by {current_ros_version} "
            "is already at its latest published version."
        )
        return

    print(f"Found {len(pending_leaf_bumps)} pending leaf bump(s):")
    for name, new_version in sorted(pending_leaf_bumps.items()):
        print(f"  {name}: {current_packages[name]} -> {new_version} (leaf, already merged via create_patch)")

    # Cascade to a fixed point: always read/increment from each package's
    # own latest published version, never from what the current ros
    # release happens to reference -- this is what guarantees N+1 is
    # relative to true latest even when the distribution's pin is stale.
    updated_modules: Dict[str, str] = dict(pending_leaf_bumps)
    changed = True
    while changed:
        changed = False
        for name in current_packages:
            if name in updated_modules:
                continue
            deps = bzlmod_lib.scan_module_for_dependencies(
                modules_dir / name / latest[name] / "MODULE.bazel", modules_dir
            )
            if any(dep in updated_modules for dep in deps):
                new_version = bzlmod_lib.increment_version(latest[name])
                updated_modules[name] = new_version
                changed = True

    cascade_only = {k: v for k, v in updated_modules.items() if k not in pending_leaf_bumps}
    if cascade_only:
        print(f"Found {len(cascade_only)} cascade-only bump(s):")
        for name, new_version in sorted(cascade_only.items()):
            print(f"  {name}: {latest[name]} -> {new_version} (cascade, bazel_dep pin update only)")

    # Materialize cascade-only packages. Leaf-bumped packages already
    # exist on disk (created by an earlier create_patch PR) -- do not
    # recreate or re-append metadata for those.
    for name, new_version in cascade_only.items():
        old_dir = modules_dir / name / latest[name]
        new_dir = modules_dir / name / new_version
        print(f"Materializing {name}@{new_version} (content unchanged from {latest[name]})...")
        if not args.dry_run:
            shutil.copytree(old_dir, new_dir, dirs_exist_ok=True)
            bzlmod_lib.add_version_to_metadata_json(modules_dir / name / "metadata.json", new_version)

    # Rewrite bazel_dep pins in every bumped package for any dependency
    # that also got bumped (leaf-bumped packages may themselves depend on
    # something else that was bumped, so their pins need rewriting too;
    # cascade-only packages additionally need their own module() version
    # stamp rewritten, since copytree alone leaves the old version string).
    for name, new_version in updated_modules.items():
        new_module_file = modules_dir / name / new_version / "MODULE.bazel"
        if name in cascade_only and args.dry_run:
            # The new_version dir only exists on disk once materialized
            # above, which --dry-run skips -- read from the still-current
            # dir instead so a dry run doesn't require files it deliberately
            # never wrote.
            content = (modules_dir / name / latest[name] / "MODULE.bazel").read_text()
        else:
            content = new_module_file.read_text()
        if name in cascade_only:
            content = bzlmod_lib.rewrite_module_version(content, name, new_version)
        for dep_name, dep_new_version in updated_modules.items():
            if dep_name == name:
                continue
            if f'name = "{dep_name}"' in content:
                content = bzlmod_lib.rewrite_bazel_dep_version(content, dep_name, dep_new_version)
        if not args.dry_run:
            new_module_file.write_text(content)

    # Regenerate the top-level ros module: copy forward (source.json is
    # byte-identical -- ros has no patches/overlay), then targeted-rewrite
    # its own version stamp and every bumped dependency's pin.
    new_ros_version = bzlmod_lib.increment_version(current_ros_version)
    print(f"Regenerating ros: {current_ros_version} -> {new_ros_version}")
    old_ros_dir = modules_dir / "ros" / current_ros_version
    new_ros_dir = modules_dir / "ros" / new_ros_version
    content = (old_ros_dir / "MODULE.bazel").read_text()
    content = bzlmod_lib.rewrite_module_version(content, "ros", new_ros_version)
    for name, new_version in updated_modules.items():
        content = bzlmod_lib.rewrite_bazel_dep_version(content, name, new_version)
    if not args.dry_run:
        shutil.copytree(old_ros_dir, new_ros_dir, dirs_exist_ok=True)
        (new_ros_dir / "MODULE.bazel").write_text(content)
        bzlmod_lib.add_version_to_metadata_json(modules_dir / "ros" / "metadata.json", new_ros_version)

    print("Bumping the CI matrix in the reusable distribution-test workflows:")
    bump_ci_matrix_files(repo_root, current_ros_version, new_ros_version, args.dry_run)

    if args.dry_run:
        print("Dry run: no files written.")
    else:
        print(f"Rolled up to ros version {new_ros_version}.")


if __name__ == "__main__":
    main()
