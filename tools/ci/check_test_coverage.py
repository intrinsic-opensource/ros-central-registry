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

"""
Compares tests run by upstream colcon builds against tests run by the Bazel
distribution build, and reports per-package coverage gaps as markdown.

Colcon JUnit results are expected under `build/<package>/test_results/**/*.xml`
(one file per gtest/ctest/pytest target) and/or `build/<package>/pytest.xml`
(a single aggregated file used by ament_python packages, where individual
tests are grouped by classname `<package>.test.<module>[.<Class>]`).

Bazel JUnit results are expected under
`bazel-testlogs/external/<package>+/<test_target>/test.xml`. `--bazel-results`
may point at an ancestor of that path (e.g. an extracted CI artifact), or
directly at a `bazel-testlogs` or `external` directory (e.g. the
`bazel-testlogs` symlink Bazel creates in a workspace).

Inputs may be given as directories, or as tar archives (.tar.gz/.tgz/.tar)
which are extracted automatically, including any archives nested inside a
given directory (as produced by `actions/download-artifact`).
"""

import argparse
import re
import shutil
import sys
import tarfile
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

# ament linters that Bazel's `@rosdistro//python:defs.bzl` py_pytest generator
# replicates per Python package, as targets named "test_<linter>" (see e.g.
# modules/launch/*/overlay/BUILD.bazel). Colcon runs the same checks but names
# them "<linter>" (ament_cmake-style xunit.xml) or "test_<linter>" (ament_python
# pytest.xml module name) depending on the package's build type. Canonicalize
# both to "test_<linter>" so they compare as the same test.
#
# cpplint/cppcheck/uncrustify/lint_cmake (the C/C++-focused linters) have no
# Bazel equivalent at all, so they're left as plain test names and will
# legitimately show up as coverage gaps.
BAZEL_REPLICATED_LINTERS = {"copyright", "flake8", "pep257", "xmllint", "mypy", "pycodestyle"}

PYTEST_CLASSNAME_RE = re.compile(r"^[^.]+\.test\.([^.]+)")

# rcl/rcl_action etc. register the same test once per rmw implementation,
# suffixing the CTest name, e.g. "test_client__rmw_fastrtps_cpp". Bazel runs
# each rmw implementation as a separate CI job instead, so its test target is
# just "test_client". Strip the suffix so these compare as the same test.
RMW_SUFFIX_RE = re.compile(r"__rmw_[a-z0-9_]+$")

def _canonicalize_test_name(name: str) -> str:
    base = name[len("test_"):] if name.startswith("test_") else name
    if base in BAZEL_REPLICATED_LINTERS:
        return f"test_{base}"
    return name

def _stem_test_name(filename: str) -> str:
    for suffix in (".gtest.xml", ".xunit.xml", ".xml"):
        if filename.endswith(suffix):
            name = RMW_SUFFIX_RE.sub("", filename[: -len(suffix)])
            return _canonicalize_test_name(name)
    return filename

def extract_archives(root: Path, max_passes: int = 5) -> None:
    """Recursively extract any tar archives found under `root`, in place."""
    for _ in range(max_passes):
        archives = [
            p for p in root.rglob("*")
            if p.is_file() and p.name.endswith((".tar.gz", ".tgz", ".tar"))
        ]
        if not archives:
            return
        for archive in archives:
            dest = archive.parent / archive.name.split(".tar")[0]
            dest.mkdir(exist_ok=True)
            with tarfile.open(archive) as tf:
                tf.extractall(dest, filter="data")
            archive.unlink()

def materialize(path: Path, work_dir: Path) -> Path:
    """Return a directory containing the fully-extracted contents of `path`,
    which may already be a directory or a tar archive."""
    dest = work_dir / path.name.replace("/", "_")
    dest.mkdir(parents=True, exist_ok=True)
    if path.is_dir():
        shutil.copytree(path, dest, dirs_exist_ok=True)
    else:
        with tarfile.open(path) as tf:
            tf.extractall(dest, filter="data")
    extract_archives(dest)
    return dest

def parse_pytest_aggregate(pytest_xml: Path) -> set[str]:
    """Extract test module names from an aggregated ament_python pytest.xml."""
    names = set()
    try:
        root = ET.parse(pytest_xml).getroot()
    except ET.ParseError:
        return names
    suites = root.findall("testsuite") if root.tag == "testsuites" else [root]
    for suite in suites:
        for testcase in suite.findall("testcase"):
            name = testcase.get("name", "")
            if name == "pytest.missing_result":
                continue
            classname = testcase.get("classname", "")
            match = PYTEST_CLASSNAME_RE.match(classname)
            module = match.group(1) if match else classname
            if module:
                names.add(_canonicalize_test_name(module))
    return names

def collect_colcon_package_tests(package_dir: Path) -> set[str]:
    """Collect the set of test names for a single `build/<package>` directory."""
    names = set()

    test_results = package_dir / "test_results"
    if test_results.is_dir():
        for xml_path in test_results.rglob("*.xml"):
            names.add(_stem_test_name(xml_path.name))

    pytest_xml = package_dir / "pytest.xml"
    if pytest_xml.is_file():
        names |= parse_pytest_aggregate(pytest_xml)

    return names

def discover_colcon_tests(root: Path) -> dict[str, set[str]]:
    """Find every `build/<package>` directory under `root` and collect its tests."""
    packages: dict[str, set[str]] = {}
    for build_dir in root.rglob("build"):
        if not build_dir.is_dir():
            continue
        for package_dir in build_dir.iterdir():
            if not package_dir.is_dir():
                continue
            tests = collect_colcon_package_tests(package_dir)
            if tests:
                packages.setdefault(package_dir.name, set()).update(tests)
    return packages

def _find_bazel_external_dirs(root: Path) -> list[Path]:
    """Locate `bazel-testlogs/external` directories under `root`.

    Handles being pointed at an ancestor of `bazel-testlogs/` (e.g. an
    extracted CI artifact or tarball), as well as being pointed directly at
    a `bazel-testlogs` or `external` directory itself (e.g. the
    `bazel-testlogs` symlink Bazel creates in a workspace), since `rglob`
    alone only matches the former.
    """
    found = {p.resolve() for p in root.rglob("bazel-testlogs/external") if p.is_dir()}
    if root.name == "bazel-testlogs" and (root / "external").is_dir():
        found.add((root / "external").resolve())
    if root.name == "external" and root.is_dir():
        found.add(root.resolve())
    return list(found)

def discover_bazel_tests(root: Path) -> dict[str, set[str]]:
    """Find every `bazel-testlogs/external/<package>+` directory under `root`."""
    packages: dict[str, set[str]] = {}
    for external_dir in _find_bazel_external_dirs(root):
        for package_dir in external_dir.iterdir():
            if not package_dir.is_dir() or not package_dir.name.endswith("+"):
                continue
            package = package_dir.name[:-1]
            tests = {
                _canonicalize_test_name(test_xml.parent.name)
                for test_xml in package_dir.glob("*/test.xml")
            }
            if tests:
                packages.setdefault(package, set()).update(tests)
    return packages

@dataclass
class PackageCoverage:
    package: str
    colcon_tests: set[str] = field(default_factory=set)
    bazel_tests: set[str] = field(default_factory=set)

    @property
    def matched(self) -> set[str]:
        return self.colcon_tests & self.bazel_tests

    @property
    def missing(self) -> set[str]:
        return self.colcon_tests - self.bazel_tests

    @property
    def extra(self) -> set[str]:
        return self.bazel_tests - self.colcon_tests

def compute_coverage(
    colcon: dict[str, set[str]], bazel: dict[str, set[str]]
) -> list[PackageCoverage]:
    coverage = []
    for package, colcon_tests in colcon.items():
        coverage.append(PackageCoverage(
            package=package,
            colcon_tests=colcon_tests,
            bazel_tests=bazel.get(package, set()),
        ))
    return sorted(coverage, key=lambda c: (-len(c.missing), c.package))

def build_full_package_list(
    colcon: dict[str, set[str]], bazel: dict[str, set[str]]
) -> list[PackageCoverage]:
    """Every package seen in either colcon or Bazel results, alphabetically."""
    packages = []
    for package in sorted(set(colcon) | set(bazel)):
        packages.append(PackageCoverage(
            package=package,
            colcon_tests=colcon.get(package, set()),
            bazel_tests=bazel.get(package, set()),
        ))
    return packages

def render_full_listing(full_packages: list[PackageCoverage]) -> list[str]:
    lines = []
    lines.append("## Full Test Listing")
    lines.append("")
    lines.append("Every test seen in either colcon or Bazel results, grouped by")
    lines.append("package. An empty cell means that test has no corresponding")
    lines.append("entry in the other system.")
    lines.append("")
    for pkg in full_packages:
        all_tests = sorted(pkg.colcon_tests | pkg.bazel_tests)
        if not all_tests:
            continue
        lines.append(f"### {pkg.package}")
        lines.append("")
        lines.append("| # | Colcon Test | Bazel Test |")
        lines.append("| --- | --- | --- |")
        for i, name in enumerate(all_tests, start=1):
            colcon_cell = name if name in pkg.colcon_tests else ""
            bazel_cell = name if name in pkg.bazel_tests else ""
            lines.append(f"| {i} | {colcon_cell} | {bazel_cell} |")
        lines.append("")
    return lines

def render_markdown(
    coverage: list[PackageCoverage],
    bazel_only: list[str],
    full_packages: list[PackageCoverage],
) -> str:
    total_colcon = sum(len(c.colcon_tests) for c in coverage)
    total_matched = sum(len(c.matched) for c in coverage)
    total_missing = sum(len(c.missing) for c in coverage)
    packages_uncovered = [c for c in coverage if not c.bazel_tests]
    packages_with_gaps = [c for c in coverage if c.bazel_tests and c.missing]
    packages_full = [c for c in coverage if c.colcon_tests and not c.missing]

    lines = []
    lines.append("# Bazel Test Coverage Report")
    lines.append("")
    lines.append("Comparison of tests run by upstream colcon builds against tests run")
    lines.append("by the Bazel distribution build.")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- Packages with colcon tests: {len(coverage)}")
    lines.append(f"- Packages with full Bazel coverage: {len(packages_full)}")
    lines.append(f"- Packages with partial Bazel coverage: {len(packages_with_gaps)}")
    lines.append(f"- Packages with no Bazel coverage at all: {len(packages_uncovered)}")
    lines.append(f"- Colcon test units: {total_colcon}")
    lines.append(f"- Matched in Bazel: {total_matched}")
    lines.append(f"- Missing from Bazel: {total_missing}")
    lines.append("")

    lines.append("## Packages with no Bazel coverage")
    lines.append("")
    if packages_uncovered:
        lines.append("| Package | Colcon tests |")
        lines.append("| --- | --- |")
        for c in sorted(packages_uncovered, key=lambda c: (-len(c.colcon_tests), c.package)):
            lines.append(f"| {c.package} | {len(c.colcon_tests)} |")
    else:
        lines.append("None.")
    lines.append("")

    lines.append("## Packages with partial Bazel coverage")
    lines.append("")
    if packages_with_gaps:
        lines.append("| Package | Colcon tests | Bazel tests | Missing |")
        lines.append("| --- | --- | --- | --- |")
        for c in packages_with_gaps:
            missing = ", ".join(sorted(c.missing))
            lines.append(f"| {c.package} | {len(c.colcon_tests)} | {len(c.bazel_tests)} | {missing} |")
    else:
        lines.append("None.")
    lines.append("")

    lines.append("## Packages with full Bazel coverage")
    lines.append("")
    if packages_full:
        lines.append(", ".join(sorted(c.package for c in packages_full)))
    else:
        lines.append("None.")
    lines.append("")

    lines.append("## Packages only seen in Bazel")
    lines.append("")
    lines.append("Packages with Bazel test results but no matching colcon results")
    lines.append("in the given input (e.g. not part of this ROS distribution).")
    lines.append("")
    if bazel_only:
        lines.append(", ".join(sorted(bazel_only)))
    else:
        lines.append("None.")
    lines.append("")

    lines.extend(render_full_listing(full_packages))

    return "\n".join(lines)

def main():
    parser = argparse.ArgumentParser(
        description="Compare colcon and Bazel test results and report coverage gaps."
    )
    parser.add_argument(
        "--colcon-results",
        required=True,
        nargs="+",
        type=Path,
        help="Paths to colcon JUnit result directories or tar archives.",
    )
    parser.add_argument(
        "--bazel-results",
        required=True,
        nargs="+",
        type=Path,
        help="Paths to Bazel JUnit result directories or tar archives.",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Path to write the markdown coverage report to.",
    )
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=None,
        help="Directory to extract archives into (default: a temp directory).",
    )
    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tmp:
        work_dir = args.work_dir or Path(tmp)
        work_dir.mkdir(parents=True, exist_ok=True)

        colcon_tests: dict[str, set[str]] = {}
        for i, path in enumerate(args.colcon_results):
            extracted = materialize(path, work_dir / f"colcon_{i}")
            for package, tests in discover_colcon_tests(extracted).items():
                colcon_tests.setdefault(package, set()).update(tests)

        bazel_tests: dict[str, set[str]] = {}
        for i, path in enumerate(args.bazel_results):
            extracted = materialize(path, work_dir / f"bazel_{i}")
            for package, tests in discover_bazel_tests(extracted).items():
                bazel_tests.setdefault(package, set()).update(tests)

        coverage = compute_coverage(colcon_tests, bazel_tests)
        bazel_only = sorted(set(bazel_tests) - set(colcon_tests))
        full_packages = build_full_package_list(colcon_tests, bazel_tests)

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(render_markdown(coverage, bazel_only, full_packages))

    print(f"Wrote coverage report to {args.output}")
    sys.exit(0)

if __name__ == "__main__":
    main()
