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

load("@rosidl_adapter//:aspects.bzl", "rosidl_adapter_aspect")
load("@rosidl_adapter//:tools.bzl", "extract_dynamic_library_runfiles_for_provider")
load("@rosidl_adapter_proto//:aspects.bzl", "rosidl_adapter_proto_aspect")
load("@rosidl_cmake//:types.bzl", "RosInterfaceInfo")
load("@rosidl_generator_c//:aspects.bzl", "rosidl_generator_c_aspect")
load("@rosidl_generator_c//:types.bzl", "RosCBindingsInfo")
load("@rosidl_generator_type_description//:aspects.bzl", "rosidl_generator_type_description_aspect")
load("@rosidl_typesupport_cpp//:aspects.bzl", "rosidl_typesupport_cpp_aspect")
load("@rosidl_typesupport_cpp//:types.bzl", "RosCcTypesupportInfo")
load("@rosidl_typesupport_fastrtps_cpp//:aspects.bzl", "rosidl_typesupport_fastrtps_cpp_aspect")
load("@rosidl_typesupport_fastrtps_cpp//:types.bzl", "RosCcTypesupportFastRTPSInfo")
load("@rosidl_typesupport_introspection_cpp//:aspects.bzl", "rosidl_typesupport_introspection_cpp_aspect")
load("@rosidl_typesupport_introspection_cpp//:types.bzl", "RosCcTypesupportIntrospectionInfo")
load("@rosidl_typesupport_protobuf_cpp//:aspects.bzl", "rosidl_typesupport_protobuf_cpp_aspect")
load("@rosidl_typesupport_protobuf_cpp//:types.bzl", "RosCcTypesupportProtobufInfo")
load("@rules_cc//cc:defs.bzl", "CcInfo", "cc_common")
load(":aspects.bzl", "rosidl_generator_cpp_aspect")
load(":types.bzl", "RosCcBindingsInfo")

DYNAMIC_TYPESUPPORTS = [
    RosCcTypesupportFastRTPSInfo,
    RosCcTypesupportIntrospectionInfo,
    RosCcTypesupportProtobufInfo,
]

def _cc_ros_library(ctx):
    default_info = extract_dynamic_library_runfiles_for_provider(ctx, DYNAMIC_TYPESUPPORTS)
    direct_cc_infos = [
        dep[RosCcTypesupportInfo].cc_info
        for dep in ctx.attr.deps
        if RosCcTypesupportInfo in dep
    ]
    if ctx.attr.static_typesupport:
        for provider in DYNAMIC_TYPESUPPORTS:
            direct_cc_infos.extend([
                dep[provider].cc_info
                for dep in ctx.attr.deps
                if provider in dep
            ])
    cc_info = cc_common.merge_cc_infos(direct_cc_infos = direct_cc_infos)
    return [default_info, cc_info]

cc_ros_library = rule(
    implementation = _cc_ros_library,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                # Adapters
                rosidl_adapter_aspect,
                rosidl_generator_type_description_aspect,
                rosidl_adapter_proto_aspect,
                # Bindings
                rosidl_generator_c_aspect,
                rosidl_generator_cpp_aspect,
                # Typesupports
                rosidl_typesupport_introspection_cpp_aspect,
                rosidl_typesupport_fastrtps_cpp_aspect,
                rosidl_typesupport_protobuf_cpp_aspect,
                rosidl_typesupport_cpp_aspect,
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
        "static_typesupport": attr.bool(default = True),
    },
    provides = [DefaultInfo, CcInfo],
)
