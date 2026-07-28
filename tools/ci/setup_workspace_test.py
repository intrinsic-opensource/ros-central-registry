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
import tempfile
import unittest
from pathlib import Path

from tools.ci import setup_workspace


class TestCalculatePackagesForVariant(unittest.TestCase):

    def setUp(self):
        self.module_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.module_dir)

    def write_module_bazel(self, name: str, version: str, deps=None):
        pkg_dir = self.module_dir / name / version
        pkg_dir.mkdir(parents=True)
        content = f'module(name = "{name}", version = "{version}")\n'
        for dep_name, dep_version in (deps or []):
            content += f'bazel_dep(name = "{dep_name}", version = "{dep_version}")\n'
        (pkg_dir / "MODULE.bazel").write_text(content)

    def test_collects_transitive_dependencies(self):
        self.write_module_bazel("pkg_a", "1.0.0", deps=[("pkg_b", "2.0.0")])
        self.write_module_bazel("pkg_b", "2.0.0", deps=[("pkg_c", "3.0.0")])
        self.write_module_bazel("pkg_c", "3.0.0")
        result = setup_workspace.calculate_packages_for_variant(
            self.module_dir, "pkg_a", "1.0.0")
        self.assertEqual(result, ["pkg_a", "pkg_b", "pkg_c"])

    def test_diamond_dependency_is_only_visited_once(self):
        self.write_module_bazel(
            "pkg_a", "1.0.0", deps=[("pkg_b", "1.0.0"), ("pkg_c", "1.0.0")])
        self.write_module_bazel("pkg_b", "1.0.0", deps=[("pkg_d", "1.0.0")])
        self.write_module_bazel("pkg_c", "1.0.0", deps=[("pkg_d", "1.0.0")])
        self.write_module_bazel("pkg_d", "1.0.0")
        result = setup_workspace.calculate_packages_for_variant(
            self.module_dir, "pkg_a", "1.0.0")
        self.assertEqual(result, ["pkg_a", "pkg_b", "pkg_c", "pkg_d"])

    def test_rosdistro_is_always_discarded(self):
        self.write_module_bazel(
            "pkg_a", "1.0.0", deps=[("rosdistro", "lyrical.2026-06-08")])
        result = setup_workspace.calculate_packages_for_variant(
            self.module_dir, "pkg_a", "1.0.0")
        self.assertEqual(result, ["pkg_a"])

    def test_dependency_with_missing_module_file_is_not_traversed_further(self):
        self.write_module_bazel(
            "pkg_a", "1.0.0", deps=[("pkg_missing", "9.9.9")])
        result = setup_workspace.calculate_packages_for_variant(
            self.module_dir, "pkg_a", "1.0.0")
        # pkg_missing has no MODULE.bazel on disk, so it's never queued and
        # never shows up as a collected package.
        self.assertEqual(result, ["pkg_a"])

    def test_start_module_with_missing_file_returns_only_itself(self):
        result = setup_workspace.calculate_packages_for_variant(
            self.module_dir, "pkg_a", "1.0.0")
        self.assertEqual(result, ["pkg_a"])


if __name__ == "__main__":
    unittest.main()
