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

load("@aspect_rules_py//py:defs.bzl", "py_library")
load("@rosidl_adapter//:aspects.bzl", "rosidl_adapter_aspect")
load("@rosidl_adapter//:tools.bzl", "extract_dynamic_library_runfiles_for_provider")
load("@rosidl_adapter_proto//:aspects.bzl", "rosidl_adapter_proto_aspect")
load("@rosidl_cmake//:types.bzl", "RosInterfaceInfo")
load("@rosidl_generator_c//:aspects.bzl", "rosidl_generator_c_aspect")
load("@rosidl_generator_cpp//:aspects.bzl", "rosidl_generator_cpp_aspect")
load("@rosidl_generator_type_description//:aspects.bzl", "rosidl_generator_type_description_aspect")
load("@rosidl_typesupport_c//:aspects.bzl", "rosidl_typesupport_c_aspect")
load("@rosidl_typesupport_fastrtps_c//:aspects.bzl", "rosidl_typesupport_fastrtps_c_aspect")
load("@rosidl_typesupport_fastrtps_c//:types.bzl", "RosCTypesupportFastRTPSInfo")
load("@rosidl_typesupport_fastrtps_cpp//:aspects.bzl", "rosidl_typesupport_fastrtps_cpp_aspect")
load("@rosidl_typesupport_fastrtps_cpp//:types.bzl", "RosCcTypesupportFastRTPSInfo")
load("@rosidl_typesupport_introspection_c//:aspects.bzl", "rosidl_typesupport_introspection_c_aspect")
load("@rosidl_typesupport_introspection_c//:types.bzl", "RosCTypesupportIntrospectionInfo")
load("@rosidl_typesupport_protobuf_c//:aspects.bzl", "rosidl_typesupport_protobuf_c_aspect")
load("@rosidl_typesupport_protobuf_c//:types.bzl", "RosCTypesupportProtobufInfo")
load("@rules_python//python:defs.bzl", "PyInfo")
load(":aspects.bzl", "rosidl_generator_py_aspect")
load(":types.bzl", "RosPyBindingsInfo")

DYNAMIC_TYPESUPPORTS = [
    RosCcTypesupportFastRTPSInfo,
    RosCTypesupportFastRTPSInfo,
    RosCTypesupportIntrospectionInfo,
    RosCTypesupportProtobufInfo,
    RosPyBindingsInfo,
]

def _py_ros_library_rule_impl(ctx):
    default_info = extract_dynamic_library_runfiles_for_provider(ctx, DYNAMIC_TYPESUPPORTS)
    py_info = PyInfo(
        imports = depset(
            transitive = [
                dep[RosPyBindingsInfo].imports
                for dep in ctx.attr.deps
                if RosPyBindingsInfo in dep
            ],
        ),
        transitive_sources = depset(
            transitive = [
                dep[RosPyBindingsInfo].transitive_sources
                for dep in ctx.attr.deps
                if RosPyBindingsInfo in dep
            ],
        ),
    )
    return [default_info, py_info]

py_ros_library_rule = rule(
    implementation = _py_ros_library_rule_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                # Adapters
                rosidl_adapter_aspect,
                rosidl_generator_type_description_aspect,
                rosidl_adapter_proto_aspect,
                # Generators
                rosidl_generator_c_aspect,
                rosidl_generator_cpp_aspect,
                # Typesupports
                rosidl_typesupport_introspection_c_aspect,
                rosidl_typesupport_fastrtps_cpp_aspect,
                rosidl_typesupport_fastrtps_c_aspect,
                rosidl_typesupport_protobuf_c_aspect,
                rosidl_typesupport_c_aspect,
                # Python
                rosidl_generator_py_aspect,
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [DefaultInfo, PyInfo],
)

def py_ros_library(name, deps):
    """
    Convenience function for setting up py dependencies.

    This is entirely a convenience wrapper to setup a dependency
    on rosidl_generator_py in order to pull in python deps (with
    c extensions and files) as well as the hook.
    """
    rule_name = "{}_internal".format(name)
    py_ros_library_rule(
        name = rule_name,
        deps = deps,
    )
    py_library(
        name = name,
        deps = [
            ":{}".format(rule_name),
            "@rosidl_generator_py",
        ],
    )
