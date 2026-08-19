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
Generates a human-readable diff for every module version a PR newly adds,
comparing it against the immediately-preceding version of the same
package -- the same reviewer experience the Bazel Central Registry gives
PRs that add a new module version (see
bazelbuild/bazel-central-registry's generate_module_diff.yml).
"""

import argparse
import os
import subprocess
from pathlib import Path
from typing import List, Optional, Tuple

from tools.ci import bzlmod_lib


def find_new_module_versions(
    diffs: List[Tuple[str, str]], target_dir: str
) -> List[Tuple[str, str]]:
    """
    Returns (package, version) pairs for every newly-added ("A" status)
    version directory under target_dir, e.g. a diff line for
    "modules/rclcpp/32.0.0-1.rcr.2/MODULE.bazel" contributes
    ("rclcpp", "32.0.0-1.rcr.2"). Deduplicated, order-preserving.
    """
    prefix = target_dir.strip("/") + "/"
    seen = []
    seen_set = set()
    for status, file_path in diffs:
        if not status.startswith("A") or not file_path.startswith(prefix):
            continue
        parts = file_path[len(prefix):].split("/")
        if len(parts) < 2:
            continue
        package, version = parts[0], parts[1]
        if version == "metadata.json":
            continue
        key = (package, version)
        if key not in seen_set:
            seen_set.add(key)
            seen.append(key)
    return seen


def find_previous_version(
    modules_dir: Path, package: str, new_version: str
) -> Optional[str]:
    """
    Finds the version immediately preceding new_version in this package's
    on-disk version history (numeric-aware), or None if new_version is the
    package's first-ever version. Every previously-published version
    directory is still present on disk (versions are immutable and
    never deleted), so this needs no access to the base ref at all.
    """
    package_dir = modules_dir / package
    versions = sorted(
        (p.name for p in package_dir.iterdir() if p.is_dir()),
        key=bzlmod_lib.version_sort_key,
    )
    if new_version not in versions:
        return None
    index = versions.index(new_version)
    return versions[index - 1] if index > 0 else None


MAX_DIFF_LINES = 300


def diff_module_versions(
    modules_dir: Path, package: str, old_version: Optional[str], new_version: str
) -> str:
    """
    Unified diff (diff -ruN) between two version directories of the same
    package -- MODULE.bazel, source.json, patches/, and overlay/ are all
    fair game to review, since they're everything a new version adds. Runs
    with cwd set to the package directory so the diff headers show clean
    version-relative paths (e.g. "1.0.0/source.json") instead of a noisy,
    non-portable absolute host path. If old_version is None (first-ever
    version of a package), diffs against an empty directory so every
    newly-added file is shown.
    """
    if old_version is None:
        import tempfile

        with tempfile.TemporaryDirectory() as empty_dir:
            result = subprocess.run(
                ["diff", "-ruN", empty_dir, new_version],
                cwd=modules_dir / package,
                capture_output=True,
                text=True,
            )
        lines = []
        for line in result.stdout.splitlines(keepends=True):
            if line.startswith("diff -ruN "):
                parts = line.strip().split()
                if len(parts) == 4:
                    lines.append(f"diff -ruN /dev/null {parts[3]}\n")
                    continue
            elif line.startswith("--- "):
                lines.append("--- /dev/null\n")
                continue
            lines.append(line)
        stdout = "".join(lines)
    else:
        result = subprocess.run(
            ["diff", "-ruN", old_version, new_version],
            cwd=modules_dir / package,
            capture_output=True,
            text=True,
        )
        stdout = result.stdout

    lines = stdout.splitlines(keepends=True)
    if len(lines) > MAX_DIFF_LINES:
        truncated = "".join(lines[:MAX_DIFF_LINES])
        truncated += f"\n... (truncated {len(lines) - MAX_DIFF_LINES} lines)\n"
        return truncated
    return stdout


def render_module_diff_markdown(
    modules_dir: Path, new_versions: List[Tuple[str, str]]
) -> str:
    """
    Renders a collapsible Markdown section per newly-added module version,
    or an empty string if there's nothing to show.
    """
    if not new_versions:
        return ""

    sections = []
    for package, version in new_versions:
        previous = find_previous_version(modules_dir, package, version)
        diff_text = diff_module_versions(modules_dir, package, previous, version)
        if not diff_text.strip():
            if previous is None:
                sections.append(
                    f"<details>\n<summary><code>{package}@{version}</code> "
                    "(first-ever version -- empty)</summary>\n</details>"
                )
            else:
                sections.append(
                    f"<details>\n<summary><code>{package}@{version}</code> "
                    f"(identical to <code>{previous}</code>)</summary>\n</details>"
                )
            continue

        if previous is None:
            summary = f"<code>{package}@{version}</code> (new module)"
        else:
            summary = (
                f"<code>{package}</code>: "
                f"<code>{previous}</code> &rarr; <code>{version}</code>"
            )

        sections.append(
            f"<details>\n<summary>{summary}</summary>\n\n"
            f"```diff\n{diff_text}```\n\n</details>"
        )

    return (
        "## \U0001F50D Module diff\n\n"
        "Diff of each new module version against the version it replaces, "
        "generated the same way the Bazel Central Registry does for new "
        "module version submissions.\n\n" + "\n\n".join(sections) + "\n"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Generate a review-friendly diff for module versions "
        "newly added by a PR, against the version each one replaces."
    )
    parser.add_argument(
        "--diff-status",
        required=True,
        type=Path,
        help="Path to the file containing git diff --name-status output",
    )
    parser.add_argument(
        "--directory",
        default="modules",
        help="Directory to scan for new module versions (e.g. modules or "
        "bcr_staging/modules)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Write the rendered Markdown here instead of stdout",
    )
    args = parser.parse_args()

    workspace_root = Path(os.environ.get("BUILD_WORKSPACE_DIRECTORY", ".")).resolve()
    modules_dir = workspace_root / args.directory

    diffs = bzlmod_lib.parse_diff_status_file(args.diff_status)
    new_versions = find_new_module_versions(diffs, args.directory)
    markdown = render_module_diff_markdown(modules_dir, new_versions)

    if args.output:
        args.output.write_text(markdown)
    else:
        print(markdown)


if __name__ == "__main__":
    main()
