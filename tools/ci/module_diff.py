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


def diff_module_versions(
    modules_dir: Path, package: str, old_version: str, new_version: str
) -> List[Tuple[str, str]]:
    """
    Compares two version directories file-by-file.
    Returns a list of (relative_file_path, diff_content) tuples.
    If there are no differences in a file, it is not included in the list.
    """
    old_dir = modules_dir / package / old_version
    new_dir = modules_dir / package / new_version

    old_files = {p.relative_to(old_dir) for p in old_dir.rglob("*") if p.is_file()}
    new_files = {p.relative_to(new_dir) for p in new_dir.rglob("*") if p.is_file()}
    all_files = sorted(old_files.union(new_files))

    file_diffs = []
    for rel_path in all_files:
        old_file = old_dir / rel_path
        new_file = new_dir / rel_path

        old_target = str(old_file) if old_file.exists() else "/dev/null"
        new_target = str(new_file) if new_file.exists() else "/dev/null"

        label_old = f"{old_version}/{rel_path}"
        label_new = f"{new_version}/{rel_path}"

        result = subprocess.run(
            ["diff", "-u", "--label", label_old, "--label", label_new, old_target, new_target],
            capture_output=True,
            text=True,
        )
        if result.stdout:
            file_diffs.append((str(rel_path), result.stdout))

    return file_diffs


MAX_FILE_DIFF_LENGTH = 10000
MAX_TOTAL_DIFF_LENGTH = 45000


def truncate_diff_text(text: str, max_chars: int) -> str:
    """
    Truncates a text string to at most max_chars, trying to align the cut
    with a line boundary.
    """
    if len(text) <= max_chars:
        return text
    truncated = text[:max_chars]
    last_newline = truncated.rfind("\n")
    if last_newline != -1:
        truncated = truncated[:last_newline]
    return truncated + "\n\n... [Diff truncated: output too large] ...\n"


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
        if previous is None:
            sections.append(
                f"<details>\n<summary><code>{package}@{version}</code> "
                "(first-ever version -- nothing to diff against)</summary>\n"
                "</details>"
            )
            continue

        file_diffs = diff_module_versions(modules_dir, package, previous, version)
        if not file_diffs:
            sections.append(
                f"<details>\n<summary><code>{package}@{version}</code> "
                f"(identical to <code>{previous}</code>)</summary>\n</details>"
            )
            continue

        module_sections = []
        total_len = 0
        limit_reached = False

        for rel_path, diff_text in file_diffs:
            if limit_reached:
                module_sections.append(
                    f"### `{rel_path}`\n\n*Diff omitted: overall limit reached.*\n"
                )
                continue

            truncated_diff = truncate_diff_text(diff_text, MAX_FILE_DIFF_LENGTH)

            # Check if adding this file's diff exceeds the total limit
            file_section = f"### `{rel_path}`\n\n```diff\n{truncated_diff}```\n"
            if total_len + len(file_section) > MAX_TOTAL_DIFF_LENGTH:
                # Truncate this file's section to fit the remaining space
                remaining = MAX_TOTAL_DIFF_LENGTH - total_len
                if remaining > 500:
                    space_for_diff = remaining - len(f"### `{rel_path}`\n\n```diff\n```\n")
                    if space_for_diff > 100:
                        truncated_diff = truncate_diff_text(diff_text, space_for_diff)
                        file_section = f"### `{rel_path}`\n\n```diff\n{truncated_diff}```\n"
                        module_sections.append(file_section)
                        total_len += len(file_section)
                    else:
                        module_sections.append(
                            f"### `{rel_path}`\n\n*Diff omitted: overall limit reached.*\n"
                        )
                else:
                    module_sections.append(
                        f"### `{rel_path}`\n\n*Diff omitted: overall limit reached.*\n"
                    )
                limit_reached = True
            else:
                module_sections.append(file_section)
                total_len += len(file_section)

        sections.append(
            f"<details>\n<summary><code>{package}</code>: "
            f"<code>{previous}</code> &rarr; <code>{version}</code></summary>\n\n"
            + "\n".join(module_sections)
            + "\n</details>"
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
