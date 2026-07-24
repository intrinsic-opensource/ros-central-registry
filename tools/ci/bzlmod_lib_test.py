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

import json
import shutil
import tempfile
import unittest
from pathlib import Path

from tools.ci import bzlmod_lib


class TestIncrementVersion(unittest.TestCase):

    def test_bumps_existing_patch(self):
        self.assertEqual(bzlmod_lib.increment_version("32.0.0-1.rcr.1"), "32.0.0-1.rcr.2")

    def test_bumps_double_digit_patch(self):
        self.assertEqual(bzlmod_lib.increment_version("32.0.0-1.rcr.9"), "32.0.0-1.rcr.10")

    def test_appends_first_patch(self):
        self.assertEqual(bzlmod_lib.increment_version("32.0.0-1"), "32.0.0-1.rcr.0")

    def test_raises_on_malformed_version(self):
        with self.assertRaises(ValueError):
            bzlmod_lib.increment_version("nodotsatall")


class TestVersionSortKey(unittest.TestCase):

    def test_numeric_aware_ordering(self):
        versions = ["32.0.0-1.rcr.10", "32.0.0-1.rcr.2", "32.0.0-1.rcr.1", "32.0.0-1"]
        self.assertEqual(
            sorted(versions, key=bzlmod_lib.version_sort_key),
            ["32.0.0-1", "32.0.0-1.rcr.1", "32.0.0-1.rcr.2", "32.0.0-1.rcr.10"],
        )


class TestMetadataJson(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.metadata_path = self.tmp_dir / "metadata.json"

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def write_metadata(self, data: dict):
        with open(self.metadata_path, "w") as f:
            json.dump(data, f)

    def test_get_latest_non_yanked_version(self):
        self.write_metadata({
            "versions": ["32.0.0-1", "32.0.0-1.rcr.1", "32.0.0-1.rcr.2"],
            "yanked_versions": {"32.0.0-1.rcr.2": "bad build"},
        })
        metadata = bzlmod_lib.read_metadata_json(self.metadata_path)
        self.assertEqual(bzlmod_lib.get_latest_non_yanked_version(metadata), "32.0.0-1.rcr.1")

    def test_get_latest_non_yanked_version_raises_if_all_yanked(self):
        self.write_metadata({
            "versions": ["1.0.0"],
            "yanked_versions": {"1.0.0": "bad"},
        })
        metadata = bzlmod_lib.read_metadata_json(self.metadata_path)
        with self.assertRaises(ValueError):
            bzlmod_lib.get_latest_non_yanked_version(metadata)

    def test_add_version_to_metadata_json_sorts_numerically(self):
        self.write_metadata({
            "versions": ["32.0.0-1.rcr.1", "32.0.0-1.rcr.9"],
            "yanked_versions": {},
        })
        bzlmod_lib.add_version_to_metadata_json(self.metadata_path, "32.0.0-1.rcr.10")
        metadata = bzlmod_lib.read_metadata_json(self.metadata_path)
        self.assertEqual(
            metadata["versions"],
            ["32.0.0-1.rcr.1", "32.0.0-1.rcr.9", "32.0.0-1.rcr.10"],
        )

    def test_add_version_to_metadata_json_is_idempotent(self):
        self.write_metadata({"versions": ["1.0.0"], "yanked_versions": {}})
        bzlmod_lib.add_version_to_metadata_json(self.metadata_path, "1.0.0")
        metadata = bzlmod_lib.read_metadata_json(self.metadata_path)
        self.assertEqual(metadata["versions"], ["1.0.0"])

    def test_read_metadata_json_raises_on_missing_file(self):
        with self.assertRaises(FileNotFoundError):
            bzlmod_lib.read_metadata_json(self.tmp_dir / "does_not_exist.json")


class TestScanModuleForDependencies(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.modules_dir = self.tmp_dir / "modules"
        (self.modules_dir / "rclcpp").mkdir(parents=True)

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def test_filters_rcr_vs_bcr(self):
        module_file = self.tmp_dir / "MODULE.bazel"
        module_file.write_text(
            'bazel_dep(name = "rclcpp", version = "32.0.0-1.rcr.1")\n'
            'bazel_dep(name = "rules_cc", version = "0.2.22")\n'
        )
        rcr_only = bzlmod_lib.scan_module_for_dependencies(module_file, self.modules_dir)
        self.assertEqual(rcr_only, {"rclcpp": "32.0.0-1.rcr.1"})
        both = bzlmod_lib.scan_module_for_dependencies(
            module_file, self.modules_dir, include_bcr=True)
        self.assertEqual(both, {"rclcpp": "32.0.0-1.rcr.1", "rules_cc": "0.2.22"})


class TestRewriteHelpers(unittest.TestCase):

    def test_rewrite_module_version(self):
        content = (
            'module(\n'
            '    name = "rclcpp",\n'
            '    version = "32.0.0-1.rcr.1",\n'
            '    bazel_compatibility = [">=7.2.1"],\n'
            ')\n'
        )
        new_content = bzlmod_lib.rewrite_module_version(content, "rclcpp", "32.0.0-1.rcr.2")
        self.assertIn('version = "32.0.0-1.rcr.2"', new_content)
        self.assertNotIn('version = "32.0.0-1.rcr.1"', new_content)

    def test_rewrite_module_version_raises_if_not_found(self):
        with self.assertRaises(RuntimeError):
            bzlmod_lib.rewrite_module_version('module(name = "other", version = "1.0.0")', "rclcpp", "2.0.0")

    def test_rewrite_bazel_dep_version(self):
        content = 'bazel_dep(name = "rclcpp", version = "32.0.0-1.rcr.1")\n'
        new_content = bzlmod_lib.rewrite_bazel_dep_version(content, "rclcpp", "32.0.0-1.rcr.2")
        self.assertEqual(new_content, 'bazel_dep(name = "rclcpp", version = "32.0.0-1.rcr.2")\n')

    def test_rewrite_bazel_dep_version_raises_if_not_found(self):
        with self.assertRaises(RuntimeError):
            bzlmod_lib.rewrite_bazel_dep_version('bazel_dep(name = "other", version = "1.0.0")', "rclcpp", "2.0.0")


class TestFindPackagesWithNewerVersions(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.modules_dir = self.tmp_dir / "modules"

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def write_metadata(self, package_name: str, versions, yanked=None):
        package_dir = self.modules_dir / package_name
        package_dir.mkdir(parents=True)
        with open(package_dir / "metadata.json", "w") as f:
            json.dump({"versions": versions, "yanked_versions": yanked or {}}, f)

    def test_detects_newer_and_ignores_up_to_date(self):
        self.write_metadata("rclcpp", ["32.0.0-1.rcr.1", "32.0.0-1.rcr.2"])
        self.write_metadata("rcl", ["10.4.4-1.rcr.1"])
        current_map = {"rclcpp": "32.0.0-1.rcr.1", "rcl": "10.4.4-1.rcr.1"}
        result = bzlmod_lib.find_packages_with_newer_versions(current_map, self.modules_dir)
        self.assertEqual(result, {"rclcpp": "32.0.0-1.rcr.2"})


if __name__ == "__main__":
    unittest.main()
