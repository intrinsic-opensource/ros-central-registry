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
load("@rosidl_cmake//:types.bzl", "RosInterfaceInfo")
load("@rosidl_generator_c//:types.bzl", "RosCBindingsInfo")
load("@rosidl_generator_cpp//:types.bzl", "RosCcBindingsInfo")
load("@rosidl_generator_type_description//:types.bzl", "RosTypeDescriptionInfo")
load("@rosidl_typesupport_fastrtps_c//:types.bzl", "RosCTypesupportFastRTPSInfo")
load("@rosidl_typesupport_fastrtps_cpp//:types.bzl", "RosCcTypesupportFastRTPSInfo")
load("@rosidl_typesupport_introspection_c//:types.bzl", "RosCTypesupportIntrospectionInfo")
load("@rosidl_typesupport_protobuf_c//:types.bzl", "RosCTypesupportProtobufInfo")
load("@rules_cc//cc:defs.bzl", "CcInfo", "cc_common")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain", "use_cc_toolchain")
load(":types.bzl", "RosCTypesupportInfo")

TYPESUPPORTS = {
    "rosidl_typesupport_fastrtps_cpp": RosCcTypesupportFastRTPSInfo,
    "rosidl_typesupport_fastrtps_c": RosCTypesupportFastRTPSInfo,
    "rosidl_typesupport_introspection_c": RosCTypesupportIntrospectionInfo,
    "rosidl_typesupport_protobuf_c": RosCTypesupportProtobufInfo,
}

def _rosidl_typesupport_c_aspect_impl(target, ctx):
    # Decide what typesupport to include based on the available providers
    additional = ["--typesupports"]
    for name, provider in TYPESUPPORTS.items():
        additional.append(name)

    # Generate the source files
    hdrs, srcs, include_dirs = generate_sources(
        target = target,
        ctx = ctx,
        executable = ctx.executable._c_typesupport_generator,
        mnemonic = "CTypeSupportGeneration",
        input_idls = [target[RosIdlInfo].idl],
        input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list(),
        input_templates = ctx.attr._c_typesupport_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [],
        templates_srcs = ["detail/{}__rosidl_typesupport_c.cpp"],
        additional = additional,
    )

    # Calculate deps for this target's CcInfo.
    deps = [dep[CcInfo] for dep in ctx.attr._c_deps if CcInfo in dep]
    deps.append(target[RosCBindingsInfo].cc_info)
    for dep in ctx.rule.attr.deps:
        if RosCTypesupportInfo in dep:
            deps.append(dep[RosCTypesupportInfo].cc_info)
    # for typesupports in TYPESUPPORTS.values():
    #     if typesupports in target:
    #         deps.append(target[typesupports].cc_info)

    cc_info, dynamic_library = generate_compilation_information(
        ctx = ctx,
        name = "{}__{}__{}__rosidl_typesupport_c".format(
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
        RosCTypesupportInfo(
            cc_info = cc_info,
            dynamic_libraries = depset(
                direct = [dynamic_library],
                transitive = [
                    dep[RosCTypesupportInfo].dynamic_libraries
                    for dep in ctx.rule.attr.deps
                    if RosCTypesupportInfo in dep
                ],
            ),
            linker_inputs = cc_info.linking_context.linker_inputs
        ),
    ]

rosidl_typesupport_c_aspect = aspect(
    implementation = _rosidl_typesupport_c_aspect_impl,
    toolchains = use_cc_toolchain(),
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        "_c_typesupport_generator": attr.label(
            default = Label("@rosidl_typesupport_c//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_c_typesupport_templates": attr.label(
            default = Label("@rosidl_typesupport_c//:interface_templates"),
        ),
        "_c_deps": attr.label_list(
            default = [
                Label("@rosidl_typesupport_c"),
            ],
            providers = [CcInfo],
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo],
        [RosCBindingsInfo],
        [RosCcBindingsInfo],
        # [RosCcTypesupportFastRTPSInfo],
        # [RosCTypesupportFastRTPSInfo],
        # [RosCTypesupportIntrospectionInfo],
        # [RosCTypesupportProtobufInfo],
    ],
    provides = [RosCTypesupportInfo],
)
