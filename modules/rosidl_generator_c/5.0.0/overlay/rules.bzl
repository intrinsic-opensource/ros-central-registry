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
load("@rosidl_generator_cpp//:aspects.bzl", "rosidl_generator_cpp_aspect")
load("@rosidl_generator_cpp//:types.bzl", "RosCcBindingsInfo")
load("@rosidl_generator_type_description//:aspects.bzl", "rosidl_generator_type_description_aspect")
load("@rosidl_typesupport_c//:aspects.bzl", "rosidl_typesupport_c_aspect")
load("@rosidl_typesupport_c//:types.bzl", "RosCTypesupportInfo")
load("@rosidl_typesupport_fastrtps_c//:aspects.bzl", "rosidl_typesupport_fastrtps_c_aspect")
load("@rosidl_typesupport_fastrtps_c//:types.bzl", "RosCTypesupportFastRTPSInfo")
load("@rosidl_typesupport_fastrtps_cpp//:aspects.bzl", "rosidl_typesupport_fastrtps_cpp_aspect")
load("@rosidl_typesupport_fastrtps_cpp//:types.bzl", "RosCcTypesupportFastRTPSInfo")
load("@rosidl_typesupport_introspection_c//:aspects.bzl", "rosidl_typesupport_introspection_c_aspect")
load("@rosidl_typesupport_introspection_c//:types.bzl", "RosCTypesupportIntrospectionInfo")
load("@rosidl_typesupport_protobuf_c//:aspects.bzl", "rosidl_typesupport_protobuf_c_aspect")
load("@rosidl_typesupport_protobuf_c//:types.bzl", "RosCTypesupportProtobufInfo")
load("@rules_cc//cc:defs.bzl", "CcInfo", "cc_common")
load(":aspects.bzl", "rosidl_generator_c_aspect")
load(":types.bzl", "RosCBindingsInfo")

DYNAMIC_TYPESUPPORTS = [
    RosCcTypesupportFastRTPSInfo,
    RosCTypesupportFastRTPSInfo,
    RosCTypesupportIntrospectionInfo,
    RosCTypesupportProtobufInfo,
]

def _c_ros_library(ctx):
    default_info = extract_dynamic_library_runfiles_for_provider(ctx, DYNAMIC_TYPESUPPORTS)
    direct_cc_infos = [
        dep[RosCTypesupportInfo].cc_info
        for dep in ctx.attr.deps
        if RosCTypesupportInfo in dep
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

c_ros_library = rule(
    implementation = _c_ros_library,
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
                rosidl_typesupport_introspection_c_aspect,
                rosidl_typesupport_fastrtps_cpp_aspect,
                rosidl_typesupport_fastrtps_c_aspect,
                rosidl_typesupport_protobuf_c_aspect,
                rosidl_typesupport_c_aspect,
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
        "static_typesupport": attr.bool(default = True),
    },
    provides = [DefaultInfo, CcInfo],
)
