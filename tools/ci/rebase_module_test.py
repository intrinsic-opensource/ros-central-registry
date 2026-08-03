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
import subprocess
import tempfile
import unittest
from pathlib import Path

from tools.ci import create_patch
from tools.ci import rebase_module
from tools.ci import vendor_modules


class TestGetCurrentRelease(unittest.TestCase):

    def setUp(self):
        self.target_workspace = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.target_workspace)

    def test_reads_release_from_module_bazel(self):
        (self.target_workspace / "MODULE.bazel").write_text(
            'bazel_dep(name = "rclcpp", version = "1.0.0")\n'
            'bazel_dep(name = "ros", version = "lyrical.2026-06-08.rcr.1")\n'
        )
        self.assertEqual(
            rebase_module.get_current_release(self.target_workspace),
            "lyrical.2026-06-08.rcr.1",
        )

    def test_raises_if_not_found(self):
        (self.target_workspace / "MODULE.bazel").write_text(
            'bazel_dep(name = "rclcpp", version = "1.0.0")\n'
        )
        with self.assertRaises(RuntimeError):
            rebase_module.get_current_release(self.target_workspace)


class TestCaptureEditDiff(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.snapshot_dir = self.tmp_dir / "snapshot"
        self.vendor_dir = self.tmp_dir / "vendor"
        self.snapshot_dir.mkdir()
        self.vendor_dir.mkdir()

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def test_matches_diff_module_source_for_non_module_bazel_files(self):
        (self.snapshot_dir / "src.c").write_text("hello\n")
        (self.vendor_dir / "src.c").write_text("goodbye\n")
        (self.vendor_dir / "new_file.h").write_text("new\n")

        patches, overlays, module_bazel = rebase_module.capture_edit_diff(
            self.snapshot_dir, self.vendor_dir)
        expected_patches, expected_overlays = create_patch.diff_module_source(
            self.snapshot_dir, self.vendor_dir)

        self.assertEqual(patches, expected_patches)
        self.assertEqual(overlays, expected_overlays)
        self.assertIsNone(module_bazel)

    def test_detects_module_bazel_change(self):
        (self.snapshot_dir / "MODULE.bazel").write_text("old\n")
        (self.vendor_dir / "MODULE.bazel").write_text("new\n")
        _, _, module_bazel = rebase_module.capture_edit_diff(self.snapshot_dir, self.vendor_dir)
        self.assertEqual(module_bazel, "new\n")

    def test_unchanged_module_bazel_is_none(self):
        (self.snapshot_dir / "MODULE.bazel").write_text("same\n")
        (self.vendor_dir / "MODULE.bazel").write_text("same\n")
        _, _, module_bazel = rebase_module.capture_edit_diff(self.snapshot_dir, self.vendor_dir)
        self.assertIsNone(module_bazel)


class TestApplyPatchText(unittest.TestCase):
    """Uses the real system `patch` CLI, consistent with the rest of this
    repo's willingness to shell out to real tools in tests."""

    def setUp(self):
        self.pristine_dir = Path(tempfile.mkdtemp())
        self.edited_dir = Path(tempfile.mkdtemp())
        self.target_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.pristine_dir)
        shutil.rmtree(self.edited_dir)
        shutil.rmtree(self.target_dir)

    def _make_patch(self, old_content: str, new_content: str) -> str:
        (self.pristine_dir / "src.c").write_text(old_content)
        (self.edited_dir / "src.c").write_text(new_content)
        patches, _ = create_patch.diff_module_source(self.pristine_dir, self.edited_dir)
        return patches["src__c.patch"]

    def test_clean_apply(self):
        patch_text = self._make_patch("line one\nline two\nline three\n", "line one\nEDITED\nline three\n")
        (self.target_dir / "src.c").write_text("line one\nline two\nline three\n")

        self.assertTrue(rebase_module.apply_patch_text(patch_text, self.target_dir))
        self.assertEqual((self.target_dir / "src.c").read_text(), "line one\nEDITED\nline three\n")

    def test_conflict_produces_reject_file(self):
        patch_text = self._make_patch("line one\nline two\nline three\n", "line one\nEDITED\nline three\n")
        # target's src.c has nothing in common with what the patch expects.
        (self.target_dir / "src.c").write_text("totally unrelated\ncontent here\nnothing matches\n")

        self.assertFalse(rebase_module.apply_patch_text(patch_text, self.target_dir))
        self.assertTrue((self.target_dir / "src.c.rej").exists())


class TestReapplyEditDiff(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def test_clean_multi_file_case(self):
        pristine_dir = self.tmp_dir / "pristine"
        edited_dir = self.tmp_dir / "edited"
        target_dir = self.tmp_dir / "target"
        pristine_dir.mkdir()
        edited_dir.mkdir()
        target_dir.mkdir()
        (pristine_dir / "src.c").write_text("line one\nline two\nline three\n")
        (target_dir / "src.c").write_text("line one\nline two\nline three\n")
        (edited_dir / "src.c").write_text("line one\nEDITED\nline three\n")
        (edited_dir / "new_file.h").write_text("new content\n")
        patches, overlays = create_patch.diff_module_source(pristine_dir, edited_dir)

        conflicting_patches, conflicting_overlays = rebase_module.reapply_edit_diff(
            patches, overlays, target_dir)

        self.assertEqual(conflicting_patches, [])
        self.assertEqual(conflicting_overlays, [])
        self.assertEqual((target_dir / "src.c").read_text(), "line one\nEDITED\nline three\n")
        self.assertEqual((target_dir / "new_file.h").read_text(), "new content\n")

    def test_overlay_collision_is_reported_but_developer_content_wins(self):
        target_dir = self.tmp_dir / "target"
        target_dir.mkdir()
        (target_dir / "new_file.h").write_text("fresh-baseline content\n")

        conflicting_patches, conflicting_overlays = rebase_module.reapply_edit_diff(
            {}, {"new_file.h": "developer content\n"}, target_dir)

        self.assertEqual(conflicting_patches, [])
        self.assertEqual(conflicting_overlays, ["new_file.h"])
        self.assertEqual((target_dir / "new_file.h").read_text(), "developer content\n")


def _fake_bazel_run(recorded_cmds, vendor_module_dir, snapshot_dir):
    """
    Fakes only the two nested `bazel run` calls rebase_module() makes
    (records the command, and for the vendor_modules call, simulates
    re-vendoring by replacing vendor_module_dir with a copy of
    snapshot_dir -- the "fresh baseline", same technique as
    vendor_modules_test.TestMain). Everything else (notably the real
    `patch` invocation apply_patch_text makes) passes through to the real
    subprocess.run unchanged.
    """
    real_run = subprocess.run

    def _fake_run(cmd, *args, **kwargs):
        if not (isinstance(cmd, list) and cmd and cmd[0] == "bazel"):
            return real_run(cmd, *args, **kwargs)
        recorded_cmds.append(cmd)
        if "//tools/ci:vendor_modules" in cmd:
            shutil.rmtree(vendor_module_dir, ignore_errors=True)
            shutil.copytree(snapshot_dir, vendor_module_dir)
        return subprocess.CompletedProcess(cmd, 0)
    return _fake_run


class TestRebaseModuleOrchestration(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.repo_root = self.tmp_dir / "repo"
        self.target_workspace = self.repo_root / "workspace"
        self.target_workspace.mkdir(parents=True)
        (self.target_workspace / "MODULE.bazel").write_text(
            'bazel_dep(name = "ros", version = "lyrical.2026-06-08.rcr.1")\n'
        )

        self.snapshot_dir = self.target_workspace / vendor_modules.VENDOR_SNAPSHOT_DIR_NAME / "rclcpp+"
        self.snapshot_dir.mkdir(parents=True)
        (self.snapshot_dir / "src.c").write_text("original\n")

        self.vendor_module_dir = self.target_workspace / "vendor" / "rclcpp+"
        self.vendor_module_dir.mkdir(parents=True)
        (self.vendor_module_dir / "src.c").write_text("developer edit\n")

        self.recorded_cmds = []
        self.original_run = subprocess.run
        rebase_module.subprocess.run = _fake_bazel_run(
            self.recorded_cmds, self.vendor_module_dir, self.snapshot_dir)

    def tearDown(self):
        rebase_module.subprocess.run = self.original_run
        shutil.rmtree(self.tmp_dir)

    def test_invokes_setup_workspace_then_vendor_modules_with_right_args(self):
        rebase_module.rebase_module("rclcpp", self.repo_root, self.target_workspace)

        self.assertEqual(len(self.recorded_cmds), 2)
        setup_cmd, vendor_cmd = self.recorded_cmds
        self.assertIn("//tools/ci:setup_workspace", setup_cmd)
        self.assertIn("--release=lyrical.2026-06-08.rcr.1", setup_cmd)
        self.assertIn("//tools/ci:vendor_modules", vendor_cmd)
        self.assertIn("rclcpp", vendor_cmd)

    def test_reapplies_edit_diff_onto_fresh_baseline_cleanly(self):
        result = rebase_module.rebase_module("rclcpp", self.repo_root, self.target_workspace)

        self.assertEqual(result.conflicting_patches, [])
        self.assertEqual(result.conflicting_overlays, [])
        self.assertEqual((self.vendor_module_dir / "src.c").read_text(), "developer edit\n")

    def test_dry_run_makes_no_subprocess_calls(self):
        rebase_module.rebase_module("rclcpp", self.repo_root, self.target_workspace, dry_run=True)
        self.assertEqual(self.recorded_cmds, [])

    def test_explicit_release_bypasses_module_bazel_lookup(self):
        rebase_module.rebase_module(
            "rclcpp", self.repo_root, self.target_workspace, release="other.release.rcr.3")
        setup_cmd, _ = self.recorded_cmds
        self.assertIn("--release=other.release.rcr.3", setup_cmd)


class TestRebaseModuleModuleBazelHandling(unittest.TestCase):

    def _setup_workspace(self, module: str):
        tmp_dir = Path(tempfile.mkdtemp())
        repo_root = tmp_dir / "repo"
        target_workspace = repo_root / "workspace"
        target_workspace.mkdir(parents=True)
        (target_workspace / "MODULE.bazel").write_text(
            'bazel_dep(name = "ros", version = "lyrical.2026-06-08.rcr.1")\n'
        )

        snapshot_dir = target_workspace / vendor_modules.VENDOR_SNAPSHOT_DIR_NAME / f"{module}+"
        snapshot_dir.mkdir(parents=True)
        (snapshot_dir / "MODULE.bazel").write_text("old module content\n")

        vendor_module_dir = target_workspace / "vendor" / f"{module}+"
        vendor_module_dir.mkdir(parents=True)
        (vendor_module_dir / "MODULE.bazel").write_text("new module content\n")

        recorded_cmds = []
        return tmp_dir, repo_root, target_workspace, _fake_bazel_run(
            recorded_cmds, vendor_module_dir, snapshot_dir)

    def _run_rebase(self, module, repo_root, target_workspace, fake_run):
        original_run = subprocess.run
        rebase_module.subprocess.run = fake_run
        try:
            return rebase_module.rebase_module(module, repo_root, target_workspace)
        finally:
            rebase_module.subprocess.run = original_run

    def test_rosdistro_module_bazel_written_aside_for_manual_reconciliation(self):
        tmp_dir, repo_root, target_workspace, fake_run = self._setup_workspace("rosdistro")
        self.addCleanup(shutil.rmtree, tmp_dir)
        result = self._run_rebase("rosdistro", repo_root, target_workspace, fake_run)

        self.assertTrue(result.module_bazel_needs_manual_reconciliation)
        self.assertEqual(result.module_bazel_your_edits_path.read_text(), "new module content\n")

    def test_non_rosdistro_module_bazel_edit_is_silently_dropped(self):
        tmp_dir, repo_root, target_workspace, fake_run = self._setup_workspace("rclcpp")
        self.addCleanup(shutil.rmtree, tmp_dir)
        result = self._run_rebase("rclcpp", repo_root, target_workspace, fake_run)

        self.assertFalse(result.module_bazel_needs_manual_reconciliation)
        self.assertIsNone(result.module_bazel_your_edits_path)


class TestRebaseModulePreconditions(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.repo_root = self.tmp_dir / "repo"
        self.target_workspace = self.repo_root / "workspace"
        self.target_workspace.mkdir(parents=True)

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def test_missing_vendor_dir_raises(self):
        with self.assertRaises(RuntimeError):
            rebase_module.rebase_module("rclcpp", self.repo_root, self.target_workspace)

    def test_missing_snapshot_raises(self):
        (self.target_workspace / "vendor" / "rclcpp+").mkdir(parents=True)
        with self.assertRaises(RuntimeError):
            rebase_module.rebase_module("rclcpp", self.repo_root, self.target_workspace)


if __name__ == "__main__":
    unittest.main()
