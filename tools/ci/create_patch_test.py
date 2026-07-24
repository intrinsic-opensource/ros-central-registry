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

import base64
import hashlib
import json
import shutil
import tarfile
import tempfile
import unittest
from pathlib import Path

from tools.ci import create_patch


class TestFetchRawUpstream(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def make_archive(self, strip_prefix: str, files: dict) -> Path:
        src_dir = self.tmp_dir / "src" / strip_prefix
        for rel_path, content in files.items():
            path = src_dir / rel_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        archive_path = self.tmp_dir / "archive.tar.gz"
        with tarfile.open(archive_path, "w:gz") as tf:
            tf.add(src_dir, arcname=strip_prefix)
        return archive_path

    def test_fetches_verifies_and_extracts(self):
        archive_path = self.make_archive("pkg-1.0.0", {"src/foo.c": "int main() {}\n"})
        digest = base64.b64encode(hashlib.sha256(archive_path.read_bytes()).digest()).decode()
        source_json_path = self.tmp_dir / "source.json"
        with open(source_json_path, "w") as f:
            json.dump({
                "url": archive_path.as_uri(),
                "strip_prefix": "pkg-1.0.0",
                "integrity": f"sha256-{digest}",
            }, f)

        dest_dir = self.tmp_dir / "dest"
        create_patch.fetch_raw_upstream(source_json_path, dest_dir)
        self.assertEqual((dest_dir / "src" / "foo.c").read_text(), "int main() {}\n")

    def test_raises_on_integrity_mismatch(self):
        archive_path = self.make_archive("pkg-1.0.0", {"src/foo.c": "content\n"})
        source_json_path = self.tmp_dir / "source.json"
        with open(source_json_path, "w") as f:
            json.dump({
                "url": archive_path.as_uri(),
                "strip_prefix": "pkg-1.0.0",
                "integrity": "sha256-wrongdigest==",
            }, f)

        with self.assertRaises(RuntimeError):
            create_patch.fetch_raw_upstream(source_json_path, self.tmp_dir / "dest")


class TestReadModuleVersion(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def test_reads_version(self):
        module_file = self.tmp_dir / "MODULE.bazel"
        module_file.write_text(
            'module(\n    name = "rclcpp",\n    version = "32.0.0-1.rcr.1",\n)\n'
        )
        self.assertEqual(create_patch.read_module_version(module_file, "rclcpp"), "32.0.0-1.rcr.1")

    def test_raises_if_not_found(self):
        module_file = self.tmp_dir / "MODULE.bazel"
        module_file.write_text('module(\n    name = "other",\n    version = "1.0.0",\n)\n')
        with self.assertRaises(RuntimeError):
            create_patch.read_module_version(module_file, "rclcpp")


class TestDiffModuleSource(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self.pristine_dir = self.tmp_dir / "pristine"
        self.edited_dir = self.tmp_dir / "edited"
        self.pristine_dir.mkdir()
        self.edited_dir.mkdir()

    def tearDown(self):
        shutil.rmtree(self.tmp_dir)

    def test_unchanged_file_produces_no_diff(self):
        (self.pristine_dir / "src.c").write_text("hello\n")
        (self.edited_dir / "src.c").write_text("hello\n")
        patches, overlays = create_patch.diff_module_source(self.pristine_dir, self.edited_dir)
        self.assertEqual(patches, {})
        self.assertEqual(overlays, {})

    def test_changed_file_produces_patch(self):
        (self.pristine_dir / "src.c").write_text("hello\n")
        (self.edited_dir / "src.c").write_text("goodbye\n")
        patches, overlays = create_patch.diff_module_source(self.pristine_dir, self.edited_dir)
        self.assertIn("src__c.patch", patches)
        self.assertIn("-hello", patches["src__c.patch"])
        self.assertIn("+goodbye", patches["src__c.patch"])
        self.assertEqual(overlays, {})

    def test_new_file_produces_overlay(self):
        (self.edited_dir / "new_file.h").write_text("new content\n")
        patches, overlays = create_patch.diff_module_source(self.pristine_dir, self.edited_dir)
        self.assertEqual(patches, {})
        self.assertEqual(overlays, {"new_file.h": "new content\n"})

    def test_deleted_file_produces_deletion_patch(self):
        (self.pristine_dir / "gone.c").write_text("bye\n")
        patches, overlays = create_patch.diff_module_source(self.pristine_dir, self.edited_dir)
        self.assertIn("gone__c.patch", patches)
        self.assertIn("-bye", patches["gone__c.patch"])
        self.assertEqual(overlays, {})

    def test_module_bazel_is_never_diffed(self):
        (self.pristine_dir / "MODULE.bazel").write_text("old\n")
        (self.edited_dir / "MODULE.bazel").write_text("new\n")
        patches, overlays = create_patch.diff_module_source(self.pristine_dir, self.edited_dir)
        self.assertEqual(patches, {})
        self.assertEqual(overlays, {})


class TestLoadExistingPatchesAndOverlays(unittest.TestCase):

    def setUp(self):
        self.module_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.module_dir)

    def test_loads_referenced_files(self):
        (self.module_dir / "patches").mkdir()
        (self.module_dir / "patches" / "foo.patch").write_text("diff content\n")
        (self.module_dir / "overlay").mkdir()
        (self.module_dir / "overlay" / "bar.h").write_text("overlay content\n")
        with open(self.module_dir / "source.json", "w") as f:
            json.dump({
                "patches": {"foo.patch": "sha256-fake"},
                "overlay": {"bar.h": "sha256-fake"},
            }, f)
        patches, overlays = create_patch.load_existing_patches_and_overlays(self.module_dir)
        self.assertEqual(patches, {"foo.patch": "diff content\n"})
        self.assertEqual(overlays, {"bar.h": "overlay content\n"})


if __name__ == "__main__":
    unittest.main()
