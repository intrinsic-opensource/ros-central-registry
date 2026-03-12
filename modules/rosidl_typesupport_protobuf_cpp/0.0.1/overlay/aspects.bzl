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

load("@rosidl_adapter//:tools.bzl", "generate_compilation_information", "generate_sources")
load("@rosidl_adapter//:types.bzl", "RosIdlInfo")
load("@rosidl_adapter_proto//:types.bzl", "RosProtoInfo")
load("@rosidl_cmake//:types.bzl", "RosInterfaceInfo")
load("@rosidl_generator_c//:types.bzl", "RosCBindingsInfo")
load("@rosidl_generator_cpp//:types.bzl", "RosCcBindingsInfo")
load("@rosidl_generator_type_description//:types.bzl", "RosTypeDescriptionInfo")
load("@rules_cc//cc:defs.bzl", "CcInfo", "cc_common")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain", "use_cc_toolchain")
load(":types.bzl", "RosCcTypesupportProtobufInfo")

def _rosidl_typesupport_protobuf_cpp_aspect_impl(target, ctx):
    hdrs, srcs, include_dirs = generate_sources(
        target = target,
        ctx = ctx,
        executable = ctx.executable._cc_typesupport_protobuf_generator,
        mnemonic = "CcTypeSupportProtobufGeneration",
        input_idls = [target[RosIdlInfo].idl],
        input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list(),
        input_templates = ctx.attr._cc_typesupport_protobuf_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [
            "{}__rosidl_typesupport_protobuf_cpp.hpp",
            "{}__typeadapter_protobuf_cpp.hpp",
        ],
        templates_srcs = [
            "detail/{}__rosidl_typesupport_protobuf_cpp.cpp",
        ],
        template_visibility_control = ctx.file._cc_typesupport_protobuf_visibility_template,
    )

    deps = [dep[CcInfo] for dep in ctx.attr._cc_deps if CcInfo in dep]
    for dep in ctx.rule.attr.deps:
        if RosCcTypesupportProtobufInfo in dep:
            deps.append(dep[RosCcTypesupportProtobufInfo].cc_info)
    deps.append(target[RosCBindingsInfo].cc_info)
    deps.append(target[RosCcBindingsInfo].cc_info)
    deps.append(target[RosProtoInfo].cc_info)

    cc_info, dynamic_library = generate_compilation_information(
        ctx = ctx,
        name = "{}__{}__{}__rosidl_typesupport_protobuf_cpp".format(
            target[RosIdlInfo].package_name,
            target[RosIdlInfo].interface_type,
            target[RosIdlInfo].interface_code,
        ),
        hdrs = hdrs,
        srcs = srcs,
        deps = deps,
        include_dirs = include_dirs,
    )

    return [
        RosCcTypesupportProtobufInfo(
            cc_info = cc_info,
            dynamic_libraries = depset(
                direct = [dynamic_library],
                transitive = [
                    dep[RosCcTypesupportProtobufInfo].dynamic_libraries
                    for dep in ctx.rule.attr.deps
                    if RosCcTypesupportProtobufInfo in dep
                ],
            ),
            linker_inputs = cc_info.linking_context.linker_inputs,
        ),
    ]

rosidl_typesupport_protobuf_cpp_aspect = aspect(
    implementation = _rosidl_typesupport_protobuf_cpp_aspect_impl,
    toolchains = use_cc_toolchain(),
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        "_cc_typesupport_protobuf_generator": attr.label(
            default = Label("@rosidl_typesupport_protobuf_cpp//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_cc_typesupport_protobuf_templates": attr.label(
            default = Label("@rosidl_typesupport_protobuf_cpp//:interface_templates"),
        ),
        "_cc_typesupport_protobuf_visibility_template": attr.label(
            default = Label("@rosidl_typesupport_protobuf_cpp//:resource/rosidl_typesupport_protobuf_cpp__visibility_control.h.in"),
            allow_single_file = True,
        ),
        "_cc_deps": attr.label_list(
            default = [
                Label("@rosidl_typesupport_protobuf_cpp"),
            ],
            providers = [CcInfo],
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosProtoInfo],
        [RosTypeDescriptionInfo],
        [RosCBindingsInfo],
        [RosCcBindingsInfo],
    ],
    provides = [RosCcTypesupportProtobufInfo],
)
