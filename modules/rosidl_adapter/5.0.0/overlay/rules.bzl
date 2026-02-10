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

load("@rosidl_cmake//:types.bzl", "RosInterfaceInfo")
load(":types.bzl", "RosIdlInfo")
load(":aspects.bzl", "rosidl_adapter_aspect")

def _idl_ros_library_impl(ctx):
    return [
        DefaultInfo(
            files = depset([
                dep[RosIdlInfo].idl
                for dep in ctx.attr.deps
                if RosIdlInfo in dep
            ])
        ),
    ]

idl_ros_library = rule(
    implementation = _idl_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [rosidl_adapter_aspect],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [DefaultInfo],
)