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

import os
import sys
import tarfile
import tempfile
import xml.etree.ElementTree as ET

def parse_junit_xml(file_path):
    """Parses a JUnit XML file and returns (tests, failures, errors, skipped)."""
    tests = 0
    failures = 0
    errors = 0
    skipped = 0
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        # JUnit XML can have <testsuites> as root, or <testsuite>
        if root.tag == 'testsuite':
            testsuites = [root]
        else:
            testsuites = root.findall('.//testsuite')
            
        for ts in testsuites:
            tests += int(ts.get('tests', 0))
            failures += int(ts.get('failures', 0))
            errors += int(ts.get('errors', 0))
            skipped += int(ts.get('skipped', 0))
            
            # If attributes are 0 but children exist, fallback to counting children
            # to be more robust
            if tests == 0:
                tests = len(ts.findall('.//testcase'))
                failures = len(ts.findall('.//testcase/failure'))
                errors = len(ts.findall('.//testcase/error'))
                skipped = len(ts.findall('.//testcase/skipped'))
    except Exception as e:
        print(f"Error parsing {file_path}: {e}", file=sys.stderr)
    return tests, failures, errors, skipped

def process_dir(directory):
    total_tests = 0
    total_failures = 0
    total_errors = 0
    total_skipped = 0
    
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith('.xml'):
                t, f, e, s = parse_junit_xml(os.path.join(root, file))
                total_tests += t
                total_failures += f
                total_errors += e
                total_skipped += s
            elif file.endswith('.tar.gz'):
                # Extract to temp directory and process
                with tempfile.TemporaryDirectory() as tmpdir:
                    try:
                        with tarfile.open(os.path.join(root, file), 'r:gz') as tar:
                            tar.extractall(path=tmpdir)
                        t, f, e, s = process_dir(tmpdir)
                        total_tests += t
                        total_failures += f
                        total_errors += e
                        total_skipped += s
                    except Exception as ex:
                        print(f"Error extracting {file}: {ex}", file=sys.stderr)
    return total_tests, total_failures, total_errors, total_skipped

def main():
    if len(sys.argv) < 2:
        print("Usage: generate_test_report.py <artifacts_dir>", file=sys.stderr)
        sys.exit(1)
        
    artifacts_dir = sys.argv[1]
    
    bazel_stats = [0, 0, 0, 0] # tests, failures, errors, skipped
    colcon_stats = [0, 0, 0, 0]
    
    if not os.path.exists(artifacts_dir):
        print(f"Directory {artifacts_dir} does not exist", file=sys.stderr)
        sys.exit(1)
        
    for item in os.listdir(artifacts_dir):
        item_path = os.path.join(artifacts_dir, item)
        if os.path.isdir(item_path):
            # Categorize based on directory name
            if "Test logs" in item:
                t, f, e, s = process_dir(item_path)
                bazel_stats[0] += t
                bazel_stats[1] += f
                bazel_stats[2] += e
                bazel_stats[3] += s
            elif "test-results-" in item:
                t, f, e, s = process_dir(item_path)
                colcon_stats[0] += t
                colcon_stats[1] += f
                colcon_stats[2] += e
                colcon_stats[3] += s
                
    # Generate markdown report
    markdown = []
    markdown.append("# Nightly Test Comparison Report")
    markdown.append("")
    markdown.append("| Build System | Total Tests | Passed | Failed | Errors | Skipped | Pass Rate |")
    markdown.append("| --- | --- | --- | --- | --- | --- | --- |")
    
    def format_row(name, stats):
        tests, failures, errors, skipped = stats
        passed = tests - failures - errors - skipped
        pass_rate = (passed / tests * 100) if tests > 0 else 0.0
        return f"| {name} | {tests} | {passed} | {failures} | {errors} | {skipped} | {pass_rate:.2f}% |"
        
    markdown.append(format_row("Bazel (RCR)", bazel_stats))
    markdown.append(format_row("Colcon (Upstream)", colcon_stats))
    markdown.append("")
    
    report_content = "\n".join(markdown)
    print(report_content)
    
    # Write to GitHub step summary if env var is set
    summary_file = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary_file:
        with open(summary_file, 'a') as f:
            f.write(report_content + "\n")

if __name__ == '__main__':
    main()
