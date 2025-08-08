#!/usr/bin/env python3

# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import argparse
import pathlib

from rosidl_adapter.cli import convert_files_to_idl
from rosidl_adapter.srv import convert_srv_to_idl


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='srv2idl',
        description='Transforms .srv files to .idl files')
    parser.add_argument('-p', '--package-path', required=False, type=pathlib.Path)
    parser.add_argument('-n', '--package-name', required=False, type=str)
    parser.add_argument('input_file', type=pathlib.Path)
    parser.add_argument('output_dir', type=pathlib.Path)
    args = parser.parse_args()
    if args.package_path is not None and args.package_name is not None:
        convert_srv_to_idl(
            package_dir = pathlib.Path.cwd() / args.package_path,
            package_name = args.package_name,
            input_file = args.input_file,
            output_dir = pathlib.Path.cwd() / args.output_dir
        )
    else:
        convert_files_to_idl('.srv', convert_srv_to_idl)
