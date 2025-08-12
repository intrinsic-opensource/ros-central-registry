# Copyright 2025 Open Source Robotics Foundation, Inc.
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

load("//ros:data.bzl", _ros_data = "ros_data")
load("//ros:interface.bzl", _RosInterfaceInfo = "RosInterfaceInfo", _ros_interface = "ros_interface")
load("//ros:package.bzl", _ros_package = "ros_package")

RosInterfaceInfo = _RosInterfaceInfo

ros_data = _ros_data
ros_interface = _ros_interface
ros_package = _ros_package


