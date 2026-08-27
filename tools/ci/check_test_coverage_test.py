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

import shutil
import tarfile
import tempfile
import unittest
from pathlib import Path

from tools.ci.check_test_coverage import (
    PackageCoverage,
    _canonicalize_test_name,
    _stem_test_name,
    build_full_package_list,
    collect_colcon_package_tests,
    compute_coverage,
    discover_bazel_tests,
    discover_colcon_tests,
    materialize,
    parse_pytest_aggregate,
    render_full_listing,
)

GTEST_XML = """<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="0" disabled="0" errors="0" time="0" name="AllTests">
  <testsuite name="Foo" tests="1" failures="0" disabled="0" skipped="0" errors="0" time="0">
    <testcase name="bar" status="run" result="completed" time="0" classname="pkg.Foo" />
  </testsuite>
</testsuites>
"""

PYTEST_AGGREGATE_XML = """<?xml version="1.0" encoding="utf-8"?>
<testsuites name="pytest tests">
  <testsuite name="pytest" errors="0" failures="0" skipped="0" tests="3" time="0.1">
    <testcase classname="mypkg.test.test_flake8" name="test_flake8" time="0" />
    <testcase classname="mypkg.test.test_thing" name="test_one" time="0" />
    <testcase classname="mypkg.test.test_thing" name="test_two" time="0" />
  </testsuite>
</testsuites>
"""

PYTEST_MISSING_RESULT_XML = """<?xml version="1.0" encoding="utf-8"?>
<testsuite name="mypkg" tests="1" failures="0" time="0" errors="1" skipped="0">
  <testcase classname="mypkg" name="pytest.missing_result" time="0" />
</testsuite>
"""

class TestHelpers(unittest.TestCase):

    def test_canonicalize_test_name(self):
        # Bazel's py_pytest linter generator replicates these six ament
        # linters per Python package, always as "test_<linter>". Colcon
        # names them "<linter>" (ament_cmake xunit) or "test_<linter>"
        # (ament_python pytest module) depending on build type -- both
        # should canonicalize to Bazel's form so they compare as one test.
        self.assertEqual(_canonicalize_test_name("copyright"), "test_copyright")
        self.assertEqual(_canonicalize_test_name("test_flake8"), "test_flake8")
        self.assertEqual(_canonicalize_test_name("pep257"), "test_pep257")

        # cpplint/cppcheck/uncrustify have no Bazel equivalent at all, so
        # they're left alone and will legitimately show up as gaps.
        self.assertEqual(_canonicalize_test_name("cppcheck_rosidl_generated_cpp"),
                          "cppcheck_rosidl_generated_cpp")
        self.assertEqual(_canonicalize_test_name("test_publisher"), "test_publisher")

    def test_stem_test_name(self):
        self.assertEqual(_stem_test_name("test_publisher.gtest.xml"), "test_publisher")
        self.assertEqual(_stem_test_name("test_client.xunit.xml"), "test_client")
        self.assertEqual(_stem_test_name("urdf_double_convert.xml"), "urdf_double_convert")

    def test_stem_test_name_strips_rmw_suffix(self):
        self.assertEqual(
            _stem_test_name("test_client__rmw_fastrtps_cpp.xml"), "test_client")
        self.assertEqual(
            _stem_test_name("test_action_communication__rmw_fastrtps_dynamic_cpp.xml"),
            "test_action_communication")

    def test_stem_test_name_canonicalizes_linter_names(self):
        self.assertEqual(_stem_test_name("copyright.xunit.xml"), "test_copyright")

class TestParsePytestAggregate(unittest.TestCase):

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_extracts_modules_and_canonicalizes_linters(self):
        path = self.tmp / "pytest.xml"
        path.write_text(PYTEST_AGGREGATE_XML)
        self.assertEqual(parse_pytest_aggregate(path), {"test_thing", "test_flake8"})

    def test_skips_missing_result_placeholder(self):
        path = self.tmp / "pytest.xml"
        path.write_text(PYTEST_MISSING_RESULT_XML)
        self.assertEqual(parse_pytest_aggregate(path), set())

class TestCollectColconPackageTests(unittest.TestCase):

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_collects_from_test_results_and_pytest_xml(self):
        package_dir = self.tmp / "rclcpp"
        (package_dir / "test_results" / "rclcpp").mkdir(parents=True)
        (package_dir / "test_results" / "rclcpp" / "test_publisher.gtest.xml").write_text(GTEST_XML)
        (package_dir / "test_results" / "rclcpp" / "copyright.xunit.xml").write_text(GTEST_XML)
        (package_dir / "pytest.xml").write_text(PYTEST_AGGREGATE_XML)

        tests = collect_colcon_package_tests(package_dir)
        # "copyright" (xunit) and "test_flake8" (pytest module) both survive,
        # canonicalized to Bazel's "test_<linter>" naming.
        self.assertEqual(tests, {"test_publisher", "test_copyright", "test_flake8", "test_thing"})

    def test_handles_flat_test_results_layout(self):
        # Some packages (e.g. urdfdom) write directly under test_results/
        # without an extra <package> subdirectory.
        package_dir = self.tmp / "urdfdom"
        (package_dir / "test_results").mkdir(parents=True)
        (package_dir / "test_results" / "urdf_double_convert.xml").write_text(GTEST_XML)

        tests = collect_colcon_package_tests(package_dir)
        self.assertEqual(tests, {"urdf_double_convert"})

class TestDiscoverColconTests(unittest.TestCase):

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_discovers_multiple_packages(self):
        build_dir = self.tmp / "build"
        for pkg, test_name in (("rclcpp", "test_publisher"), ("cv_bridge", "utest")):
            results = build_dir / pkg / "test_results" / pkg
            results.mkdir(parents=True)
            (results / f"{test_name}.gtest.xml").write_text(GTEST_XML)

        # A package with only linter results is still tracked (Bazel
        # replicates some of these per-package via py_pytest), not skipped.
        lint_only = build_dir / "std_msgs" / "test_results" / "std_msgs"
        lint_only.mkdir(parents=True)
        (lint_only / "copyright.xunit.xml").write_text(GTEST_XML)

        packages = discover_colcon_tests(self.tmp)
        self.assertEqual(packages, {
            "rclcpp": {"test_publisher"},
            "cv_bridge": {"utest"},
            "std_msgs": {"test_copyright"},
        })

class TestDiscoverBazelTests(unittest.TestCase):

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_discovers_packages_and_strips_plus_suffix(self):
        external = self.tmp / "bazel-testlogs" / "external"
        (external / "rclcpp+" / "test_publisher").mkdir(parents=True)
        (external / "rclcpp+" / "test_publisher" / "test.xml").write_text(GTEST_XML)
        (external / "rclcpp+" / "test_service").mkdir(parents=True)
        (external / "rclcpp+" / "test_service" / "test.xml").write_text(GTEST_XML)

        packages = discover_bazel_tests(self.tmp)
        self.assertEqual(packages, {"rclcpp": {"test_publisher", "test_service"}})

    def test_discovers_when_pointed_directly_at_bazel_testlogs_dir(self):
        # Bazel creates a `bazel-testlogs` symlink directly in the workspace;
        # pointing --bazel-results at it (rather than its parent) must work.
        bazel_testlogs = self.tmp / "bazel-testlogs"
        (bazel_testlogs / "external" / "rclcpp+" / "test_publisher").mkdir(parents=True)
        (bazel_testlogs / "external" / "rclcpp+" / "test_publisher" / "test.xml").write_text(GTEST_XML)

        packages = discover_bazel_tests(bazel_testlogs)
        self.assertEqual(packages, {"rclcpp": {"test_publisher"}})

    def test_discovers_when_pointed_directly_at_external_dir(self):
        external = self.tmp / "bazel-testlogs" / "external"
        (external / "rclcpp+" / "test_publisher").mkdir(parents=True)
        (external / "rclcpp+" / "test_publisher" / "test.xml").write_text(GTEST_XML)

        packages = discover_bazel_tests(external)
        self.assertEqual(packages, {"rclcpp": {"test_publisher"}})

class TestMaterialize(unittest.TestCase):

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_extracts_tar_archive(self):
        src = self.tmp / "src"
        (src / "build" / "rclcpp" / "test_results" / "rclcpp").mkdir(parents=True)
        (src / "build" / "rclcpp" / "test_results" / "rclcpp" / "test_publisher.gtest.xml").write_text(GTEST_XML)

        archive = self.tmp / "results.tar.gz"
        with tarfile.open(archive, "w:gz") as tf:
            tf.add(src / "build", arcname="build")

        work_dir = self.tmp / "work"
        extracted = materialize(archive, work_dir)
        packages = discover_colcon_tests(extracted)
        self.assertEqual(packages, {"rclcpp": {"test_publisher"}})

    def test_materialize_then_discover_bazel_tests_when_pointed_at_testlogs_dir(self):
        # End-to-end regression test: `bazel run //tools/ci:check_test_coverage
        # -- --bazel-results <path/to/bazel-testlogs>` (a real local Bazel
        # workspace's `bazel-testlogs` symlink, not a CI artifact tarball)
        # must still find tests after materialize() copies it into work_dir.
        bazel_testlogs = self.tmp / "bazel-testlogs"
        (bazel_testlogs / "external" / "rclcpp+" / "test_publisher").mkdir(parents=True)
        (bazel_testlogs / "external" / "rclcpp+" / "test_publisher" / "test.xml").write_text(GTEST_XML)

        work_dir = self.tmp / "work"
        extracted = materialize(bazel_testlogs, work_dir)
        packages = discover_bazel_tests(extracted)
        self.assertEqual(packages, {"rclcpp": {"test_publisher"}})

    def test_extracts_nested_archive_inside_directory(self):
        # Mirrors actions/download-artifact output: a directory containing a
        # nested tarball with the real results.
        src = self.tmp / "src"
        (src / "build" / "rclcpp" / "test_results" / "rclcpp").mkdir(parents=True)
        (src / "build" / "rclcpp" / "test_results" / "rclcpp" / "test_publisher.gtest.xml").write_text(GTEST_XML)

        artifact_dir = self.tmp / "artifact"
        artifact_dir.mkdir()
        with tarfile.open(artifact_dir / "results.tar.gz", "w:gz") as tf:
            tf.add(src / "build", arcname="build")

        work_dir = self.tmp / "work"
        extracted = materialize(artifact_dir, work_dir)
        packages = discover_colcon_tests(extracted)
        self.assertEqual(packages, {"rclcpp": {"test_publisher"}})

class TestLinterCoverageMatches(unittest.TestCase):
    """Regression test for the `launch` py_pytest linter targets: Bazel names
    them "test_copyright" (modules/launch/*/overlay/BUILD.bazel); colcon's
    ament_cmake xunit files name the same check bare "copyright". These must
    match rather than showing up as a false gap."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_ament_cmake_style_colcon_name_matches_bazel_target(self):
        colcon_dir = self.tmp / "colcon" / "build" / "launch" / "test_results" / "launch"
        colcon_dir.mkdir(parents=True)
        (colcon_dir / "copyright.xunit.xml").write_text(GTEST_XML)

        bazel_dir = self.tmp / "bazel" / "bazel-testlogs" / "external" / "launch+" / "test_copyright"
        bazel_dir.mkdir(parents=True)
        (bazel_dir / "test.xml").write_text(GTEST_XML)

        colcon = discover_colcon_tests(self.tmp / "colcon")
        bazel = discover_bazel_tests(self.tmp / "bazel")
        coverage = compute_coverage(colcon, bazel)

        launch = next(c for c in coverage if c.package == "launch")
        self.assertEqual(launch.matched, {"test_copyright"})
        self.assertEqual(launch.missing, set())

class TestComputeCoverage(unittest.TestCase):

    def test_matched_missing_and_extra(self):
        colcon = {
            "rclcpp": {"test_publisher", "test_service", "test_flaky_upstream_only"},
            "cv_bridge": {"utest"},
        }
        bazel = {
            "rclcpp": {"test_publisher", "test_service", "test_bazel_only"},
        }

        coverage = compute_coverage(colcon, bazel)
        by_pkg = {c.package: c for c in coverage}

        self.assertEqual(by_pkg["rclcpp"].matched, {"test_publisher", "test_service"})
        self.assertEqual(by_pkg["rclcpp"].missing, {"test_flaky_upstream_only"})
        self.assertEqual(by_pkg["rclcpp"].extra, {"test_bazel_only"})

        self.assertEqual(by_pkg["cv_bridge"].bazel_tests, set())
        self.assertEqual(by_pkg["cv_bridge"].missing, {"utest"})

        # Packages with the most missing tests should sort first.
        self.assertEqual(coverage[0].package, "cv_bridge")

class TestBuildFullPackageList(unittest.TestCase):

    def test_includes_bazel_only_packages(self):
        colcon = {"rclcpp": {"test_publisher"}}
        bazel = {
            "rclcpp": {"test_publisher"},
            "osrf_testing_tools_cpp": {"test_memory_tools"},
        }

        packages = build_full_package_list(colcon, bazel)

        self.assertEqual([p.package for p in packages], ["osrf_testing_tools_cpp", "rclcpp"])
        osrf = packages[0]
        self.assertEqual(osrf.colcon_tests, set())
        self.assertEqual(osrf.bazel_tests, {"test_memory_tools"})

class TestRenderFullListing(unittest.TestCase):

    def test_rows_restart_numbering_per_package_and_leave_gaps_empty(self):
        packages = build_full_package_list(
            colcon={"rclcpp": {"test_publisher", "test_only_in_colcon"}},
            bazel={"rclcpp": {"test_publisher", "test_only_in_bazel"}},
        )

        lines = render_full_listing(packages)
        text = "\n".join(lines)

        self.assertIn("### rclcpp", text)
        # Alphabetical within the package: test_only_in_bazel, test_only_in_colcon, test_publisher
        self.assertIn("| 1 |  | test_only_in_bazel |", text)
        self.assertIn("| 2 | test_only_in_colcon |  |", text)
        self.assertIn("| 3 | test_publisher | test_publisher |", text)

    def test_skips_packages_with_no_tests(self):
        packages = [PackageCoverage(package="empty_pkg")]

        lines = render_full_listing(packages)
        self.assertNotIn("empty_pkg", "\n".join(lines))

if __name__ == "__main__":
    unittest.main()
