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

import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

from tools.ci import module_diff


class TestFindNewModuleVersions(unittest.TestCase):

    def test_collects_added_version_directories(self):
        diffs = [
            ("A", "modules/rclcpp/32.0.0-1.rcr.2/MODULE.bazel"),
            ("A", "modules/rclcpp/32.0.0-1.rcr.2/source.json"),
            ("A", "modules/rclcpp/32.0.0-1.rcr.2/patches/0001-fix.patch"),
        ]
        result = module_diff.find_new_module_versions(diffs, "modules")
        self.assertEqual(result, [("rclcpp", "32.0.0-1.rcr.2")])

    def test_ignores_non_added_statuses(self):
        diffs = [
            ("M", "modules/rclcpp/metadata.json"),
            ("D", "modules/rclcpp/32.0.0-1.rcr.1/MODULE.bazel"),
        ]
        self.assertEqual(module_diff.find_new_module_versions(diffs, "modules"), [])

    def test_ignores_metadata_json(self):
        diffs = [("A", "modules/rclcpp/metadata.json")]
        self.assertEqual(module_diff.find_new_module_versions(diffs, "modules"), [])

    def test_ignores_files_outside_target_dir(self):
        diffs = [("A", "bcr_staging/modules/rclcpp/1.0.0/MODULE.bazel")]
        self.assertEqual(module_diff.find_new_module_versions(diffs, "modules"), [])

    def test_dedupes_and_preserves_order(self):
        diffs = [
            ("A", "modules/rclcpp/32.0.0-1.rcr.2/MODULE.bazel"),
            ("A", "modules/rcutils/7.1.1-3.rcr.0/MODULE.bazel"),
            ("A", "modules/rclcpp/32.0.0-1.rcr.2/source.json"),
        ]
        result = module_diff.find_new_module_versions(diffs, "modules")
        self.assertEqual(
            result, [("rclcpp", "32.0.0-1.rcr.2"), ("rcutils", "7.1.1-3.rcr.0")])

    def test_respects_matrix_directory(self):
        diffs = [("A", "bcr_staging/modules/foo/1.0.0/MODULE.bazel")]
        result = module_diff.find_new_module_versions(diffs, "bcr_staging/modules")
        self.assertEqual(result, [("foo", "1.0.0")])


class TestFindPreviousVersion(unittest.TestCase):

    def setUp(self):
        self.modules_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.modules_dir)

    def make_versions(self, package: str, versions):
        for version in versions:
            (self.modules_dir / package / version).mkdir(parents=True)

    def test_finds_immediately_preceding_version(self):
        self.make_versions("rclcpp", ["32.0.0-1.rcr.1", "32.0.0-1.rcr.2", "32.0.0-1.rcr.9"])
        result = module_diff.find_previous_version(self.modules_dir, "rclcpp", "32.0.0-1.rcr.9")
        self.assertEqual(result, "32.0.0-1.rcr.2")

    def test_numeric_aware_ordering(self):
        self.make_versions("rclcpp", ["32.0.0-1.rcr.1", "32.0.0-1.rcr.2", "32.0.0-1.rcr.10"])
        result = module_diff.find_previous_version(self.modules_dir, "rclcpp", "32.0.0-1.rcr.10")
        self.assertEqual(result, "32.0.0-1.rcr.2")

    def test_first_ever_version_has_no_predecessor(self):
        self.make_versions("new_pkg", ["0.0.0"])
        result = module_diff.find_previous_version(self.modules_dir, "new_pkg", "0.0.0")
        self.assertIsNone(result)


class TestDiffModuleVersions(unittest.TestCase):

    def setUp(self):
        self.modules_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.modules_dir)

    def test_shows_added_changed_and_removed_files(self):
        old_dir = self.modules_dir / "rclcpp" / "1.0.0"
        new_dir = self.modules_dir / "rclcpp" / "1.0.1"
        (old_dir / "patches").mkdir(parents=True)
        (new_dir / "patches").mkdir(parents=True)

        (old_dir / "MODULE.bazel").write_text('version = "1.0.0"\n')
        (new_dir / "MODULE.bazel").write_text('version = "1.0.1"\n')
        (old_dir / "patches" / "0001-old.patch").write_text("old patch\n")
        (new_dir / "patches" / "0002-new.patch").write_text("new patch\n")

        diff_text = module_diff.diff_module_versions(self.modules_dir, "rclcpp", "1.0.0", "1.0.1")
        self.assertIn("-version = \"1.0.0\"", diff_text)
        self.assertIn("+version = \"1.0.1\"", diff_text)
        self.assertIn("0001-old.patch", diff_text)
        self.assertIn("0002-new.patch", diff_text)

    def test_identical_versions_produce_empty_diff(self):
        old_dir = self.modules_dir / "rclcpp" / "1.0.0"
        new_dir = self.modules_dir / "rclcpp" / "1.0.1"
        old_dir.mkdir(parents=True)
        new_dir.mkdir(parents=True)
        (old_dir / "MODULE.bazel").write_text("same\n")
        (new_dir / "MODULE.bazel").write_text("same\n")

        diff_text = module_diff.diff_module_versions(self.modules_dir, "rclcpp", "1.0.0", "1.0.1")
        self.assertEqual(diff_text.strip(), "")


    def test_diff_module_versions_none_old_version(self):
        new_dir = self.modules_dir / "new_pkg" / "1.0.0"
        (new_dir / "patches").mkdir(parents=True)
        (new_dir / "MODULE.bazel").write_text('version = "1.0.0"\n')
        (new_dir / "patches" / "0001.patch").write_text("patch\n")

        diff_text = module_diff.diff_module_versions(self.modules_dir, "new_pkg", None, "1.0.0")
        self.assertIn("+version = \"1.0.0\"", diff_text)
        self.assertIn("0001.patch", diff_text)
        self.assertIn("/dev/null", diff_text)


class TestRenderModuleDiffMarkdown(unittest.TestCase):

    def setUp(self):
        self.modules_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.modules_dir)

    def test_no_new_versions_renders_nothing(self):
        self.assertEqual(module_diff.render_module_diff_markdown(self.modules_dir, []), "")

    def test_first_ever_version_renders_new_module_diff(self):
        new_dir = self.modules_dir / "new_pkg" / "0.0.0"
        new_dir.mkdir(parents=True)
        (new_dir / "MODULE.bazel").write_text('module(name = "new_pkg", version = "0.0.0")\n')
        result = module_diff.render_module_diff_markdown(self.modules_dir, [("new_pkg", "0.0.0")])
        self.assertIn("new_pkg@0.0.0", result)
        self.assertIn("new module", result)
        self.assertIn("```diff", result)

    def test_real_change_renders_diff_fence(self):
        old_dir = self.modules_dir / "rclcpp" / "1.0.0"
        new_dir = self.modules_dir / "rclcpp" / "1.0.1"
        old_dir.mkdir(parents=True)
        new_dir.mkdir(parents=True)
        (old_dir / "MODULE.bazel").write_text('version = "1.0.0"\n')
        (new_dir / "MODULE.bazel").write_text('version = "1.0.1"\n')

        result = module_diff.render_module_diff_markdown(self.modules_dir, [("rclcpp", "1.0.1")])
        self.assertIn("```diff", result)
        self.assertIn("1.0.0", result)
        self.assertIn("1.0.1", result)
        self.assertIn("<details>", result)

    def test_identical_versions_notes_no_diff(self):
        old_dir = self.modules_dir / "rclcpp" / "1.0.0"
        new_dir = self.modules_dir / "rclcpp" / "1.0.1"
        old_dir.mkdir(parents=True)
        new_dir.mkdir(parents=True)
        (old_dir / "MODULE.bazel").write_text("same\n")
        (new_dir / "MODULE.bazel").write_text("same\n")

        result = module_diff.render_module_diff_markdown(self.modules_dir, [("rclcpp", "1.0.1")])
        self.assertIn("identical to", result)
        self.assertNotIn("```diff", result)


class TestMain(unittest.TestCase):

    def setUp(self):
        self.workspace_root = Path(tempfile.mkdtemp())
        self.modules_dir = self.workspace_root / "modules"
        self.args_dir = Path(tempfile.mkdtemp())
        self.diff_status_path = self.args_dir / "diff_status.txt"
        self.output_path = self.args_dir / "module_diff.md"

        self.original_environ = dict(os.environ)
        self.original_argv = sys.argv

    def tearDown(self):
        os.environ.clear()
        os.environ.update(self.original_environ)
        sys.argv = self.original_argv
        shutil.rmtree(self.workspace_root)
        shutil.rmtree(self.args_dir)

    def test_writes_rendered_markdown_via_build_workspace_directory(self):
        old_dir = self.modules_dir / "rclcpp" / "1.0.0"
        new_dir = self.modules_dir / "rclcpp" / "1.0.1"
        old_dir.mkdir(parents=True)
        new_dir.mkdir(parents=True)
        (old_dir / "MODULE.bazel").write_text('version = "1.0.0"\n')
        (new_dir / "MODULE.bazel").write_text('version = "1.0.1"\n')
        self.diff_status_path.write_text("A\tmodules/rclcpp/1.0.1/MODULE.bazel\n")

        os.environ["BUILD_WORKSPACE_DIRECTORY"] = str(self.workspace_root)
        sys.argv = [
            "module_diff.py",
            "--diff-status", str(self.diff_status_path),
            "--directory", "modules",
            "--output", str(self.output_path),
        ]
        module_diff.main()

        content = self.output_path.read_text()
        self.assertIn("rclcpp", content)
        self.assertIn("```diff", content)

    def test_no_new_versions_writes_empty_output(self):
        self.modules_dir.mkdir(parents=True)
        self.diff_status_path.write_text("M\tmodules/rclcpp/metadata.json\n")

        os.environ["BUILD_WORKSPACE_DIRECTORY"] = str(self.workspace_root)
        sys.argv = [
            "module_diff.py",
            "--diff-status", str(self.diff_status_path),
            "--directory", "modules",
            "--output", str(self.output_path),
        ]
        module_diff.main()

        self.assertEqual(self.output_path.read_text(), "")


if __name__ == "__main__":
    unittest.main()
