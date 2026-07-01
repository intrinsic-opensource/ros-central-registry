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
from tools.ci.check_pr_modules import check_violations

class TestCheckPrModules(unittest.TestCase):

    def setUp(self):
        # Create temp directories for old metadata files and current workspace files
        self.old_dir = Path(tempfile.mkdtemp())
        self.work_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        # Clean up temp directories
        shutil.rmtree(self.old_dir)
        shutil.rmtree(self.work_dir)

    def write_json(self, base_dir: Path, rel_path: str, data: dict):
        path = base_dir / rel_path
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f)

    def test_allowed_adds_modules(self):
        # Adding new files (status A) under modules/ is allowed
        diffs = [
            ("A", "modules/new_module/metadata.json"),
            ("A", "modules/new_module/1.0.0/MODULE.bazel"),
            ("A", "modules/new_module/1.0.0/source.json"),
        ]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "modules")
        self.assertEqual(violations, [])

    def test_allowed_adds_bcr_staging(self):
        # Adding new files (status A) under bcr_staging/modules/ is allowed
        diffs = [
            ("A", "bcr_staging/modules/new_module/metadata.json"),
            ("A", "bcr_staging/modules/new_module/1.0.0/MODULE.bazel"),
            ("A", "bcr_staging/modules/new_module/1.0.0/source.json"),
        ]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "bcr_staging/modules")
        self.assertEqual(violations, [])

    def test_deleted_file_forbidden(self):
        # Deleting existing files under modules/ is forbidden
        diffs = [
            ("D", "modules/existing_module/1.0.0/MODULE.bazel")
        ]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "modules")
        self.assertIn("Deleted file in modules/: modules/existing_module/1.0.0/MODULE.bazel", violations)

    def test_renamed_file_forbidden(self):
        # Renaming existing files under modules/ is forbidden
        diffs = [
            ("R100", "modules/existing_module/1.0.0/MODULE.bazel")
        ]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "modules")
        self.assertIn("Renamed file in modules/: modules/existing_module/1.0.0/MODULE.bazel", violations)

    def test_modified_non_metadata_forbidden(self):
        # Modifying non-metadata.json files is forbidden
        diffs = [
            ("M", "modules/existing_module/1.0.0/MODULE.bazel")
        ]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "modules")
        self.assertIn("Modified existing file (only metadata.json files can be modified): modules/existing_module/1.0.0/MODULE.bazel", violations)

    def test_modified_metadata_allowed_add_version(self):
        # Modifying metadata.json to add a new version is allowed
        rel_path = "modules/existing_module/metadata.json"
        
        # Base metadata has version "1.0.0"
        self.write_json(self.old_dir, rel_path, {"versions": ["1.0.0"], "yanked_versions": {}})
        # Working metadata has "1.0.0" and "1.1.0"
        self.write_json(self.work_dir, rel_path, {"versions": ["1.0.0", "1.1.0"], "yanked_versions": {}})

        diffs = [("M", rel_path)]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "modules")
        self.assertEqual(violations, [])

    def test_modified_metadata_removed_version_forbidden(self):
        # Modifying metadata.json and removing a version without yanking is forbidden
        rel_path = "modules/existing_module/metadata.json"
        
        # Base metadata has versions "1.0.0" and "1.1.0"
        self.write_json(self.old_dir, rel_path, {"versions": ["1.0.0", "1.1.0"], "yanked_versions": {}})
        # Working metadata removes "1.0.0" without yanking it
        self.write_json(self.work_dir, rel_path, {"versions": ["1.1.0"], "yanked_versions": {}})

        diffs = [("M", rel_path)]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "modules")
        self.assertIn("Version '1.0.0' was removed from 'versions' in modules/existing_module/metadata.json but was not added to 'yanked_versions'.", violations)

    def test_modified_metadata_yanked_version_allowed(self):
        # Modifying metadata.json and moving a version to yanked_versions is allowed
        rel_path = "modules/existing_module/metadata.json"
        
        # Base metadata has versions "1.0.0" and "1.1.0"
        self.write_json(self.old_dir, rel_path, {"versions": ["1.0.0", "1.1.0"], "yanked_versions": {}})
        # Working metadata removes "1.0.0" from versions and adds it to yanked_versions
        self.write_json(self.work_dir, rel_path, {"versions": ["1.1.0"], "yanked_versions": {"1.0.0": "security flaw"}})

        diffs = [("M", rel_path)]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "modules")
        self.assertEqual(violations, [])

    def test_bcr_staging_rules(self):
        # Test that same rules apply under bcr_staging/modules directory
        rel_path = "bcr_staging/modules/existing_module/metadata.json"
        
        self.write_json(self.old_dir, rel_path, {"versions": ["1.0.0"], "yanked_versions": {}})
        self.write_json(self.work_dir, rel_path, {"versions": ["1.1.0"], "yanked_versions": {}})

        diffs = [("M", rel_path)]
        violations = check_violations(diffs, self.old_dir, self.work_dir, "bcr_staging/modules")
        self.assertIn("Version '1.0.0' was removed from 'versions' in bcr_staging/modules/existing_module/metadata.json but was not added to 'yanked_versions'.", violations)

if __name__ == "__main__":
    unittest.main()
