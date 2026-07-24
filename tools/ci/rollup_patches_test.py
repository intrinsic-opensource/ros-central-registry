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

from tools.ci import rollup_patches


class TestFindCurrentRosVersion(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.ros_dir = self.tmp_dir / "ros"
        self.ros_dir.mkdir()

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def make_release_dirs(self, names):
        for name in names:
            (self.ros_dir / name).mkdir()

    def test_finds_highest_patch(self):
        self.make_release_dirs([
            "lyrical.2026-06-08.rcr.0",
            "lyrical.2026-06-08.rcr.1",
            "lyrical.2026-06-08.rcr.9",
            "lyrical.2026-06-08.rcr.10",
        ])
        result = rollup_patches.find_current_ros_version(self.tmp_dir, "lyrical", "2026-06-08")
        self.assertEqual(result, "lyrical.2026-06-08.rcr.10")

    def test_ignores_other_distro_date_rows(self):
        self.make_release_dirs(["rolling.2026-01-21.rcr.5", "lyrical.2026-06-08.rcr.1"])
        result = rollup_patches.find_current_ros_version(self.tmp_dir, "lyrical", "2026-06-08")
        self.assertEqual(result, "lyrical.2026-06-08.rcr.1")

    def test_raises_if_none_found(self):
        with self.assertRaises(RuntimeError):
            rollup_patches.find_current_ros_version(self.tmp_dir, "lyrical", "2026-06-08")


class TestCheckNoYankedDependencies(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def write_metadata(self, package_name: str, yanked=None):
        package_dir = self.tmp_dir / package_name
        package_dir.mkdir()
        with open(package_dir / "metadata.json", "w") as f:
            json.dump({"versions": ["1.0.0"], "yanked_versions": yanked or {}}, f)

    def test_passes_when_nothing_yanked(self):
        self.write_metadata("rclcpp")
        rollup_patches.check_no_yanked_dependencies({"rclcpp": "1.0.0"}, self.tmp_dir)

    def test_raises_when_pinned_version_is_yanked(self):
        self.write_metadata("rclcpp", yanked={"1.0.0": "bad build"})
        with self.assertRaises(RuntimeError):
            rollup_patches.check_no_yanked_dependencies({"rclcpp": "1.0.0"}, self.tmp_dir)


class TestBumpCiMatrixFiles(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.workflows_dir = self.tmp_dir / ".github" / "workflows"
        self.workflows_dir.mkdir(parents=True)
        for name in rollup_patches.WORKFLOW_FILES_WITH_HARDCODED_ROS_MATRIX:
            path = self.tmp_dir / name
            path.write_text("matrix:\n  ros:\n    - lyrical.2026-06-08.rcr.1\n")

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def test_bumps_all_three_files(self):
        rollup_patches.bump_ci_matrix_files(
            self.tmp_dir, "lyrical.2026-06-08.rcr.1", "lyrical.2026-06-08.rcr.2", dry_run=False)
        for name in rollup_patches.WORKFLOW_FILES_WITH_HARDCODED_ROS_MATRIX:
            content = (self.tmp_dir / name).read_text()
            self.assertIn("- lyrical.2026-06-08.rcr.2", content)
            self.assertNotIn("- lyrical.2026-06-08.rcr.1", content)

    def test_dry_run_writes_nothing(self):
        rollup_patches.bump_ci_matrix_files(
            self.tmp_dir, "lyrical.2026-06-08.rcr.1", "lyrical.2026-06-08.rcr.2", dry_run=True)
        for name in rollup_patches.WORKFLOW_FILES_WITH_HARDCODED_ROS_MATRIX:
            content = (self.tmp_dir / name).read_text()
            self.assertIn("- lyrical.2026-06-08.rcr.1", content)

    def test_raises_if_old_entry_not_found(self):
        with self.assertRaises(RuntimeError):
            rollup_patches.bump_ci_matrix_files(
                self.tmp_dir, "not.the-right.rcr.1", "lyrical.2026-06-08.rcr.2", dry_run=False)


if __name__ == "__main__":
    unittest.main()
