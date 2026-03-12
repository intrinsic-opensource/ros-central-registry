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
load("@rosidl_generator_type_description//:types.bzl", "RosTypeDescriptionInfo")
load("@rosidl_typesupport_c//:types.bzl", "RosCTypesupportInfo")
load("@rules_cc//cc:defs.bzl", "CcInfo", "cc_common")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain", "use_cc_toolchain")
load(":types.bzl", "RosCTypesupportIntrospectionInfo")

def _rosidl_typesupport_introspection_c_aspect_impl(target, ctx):
    hdrs, srcs, include_dirs = generate_sources(
        target = target,
        ctx = ctx,
        executable = ctx.executable._c_typesupport_introspection_generator,
        mnemonic = "CTypeSupportIntrospectionGeneration",
        input_idls = [target[RosIdlInfo].idl],
        input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list(),
        input_templates = ctx.attr._c_typesupport_introspection_templates[DefaultInfo].files.to_list(),
        templates_hdrs = ["detail/{}__rosidl_typesupport_introspection_c.h"],
        templates_srcs = ["detail/{}__rosidl_typesupport_introspection_c.c"],
        template_visibility_control = ctx.file._c_typesupport_introspection_visibility_template,
    )

    deps = [dep[CcInfo] for dep in ctx.attr._c_deps if CcInfo in dep]
    for dep in ctx.rule.attr.deps:
        if RosCTypesupportIntrospectionInfo in dep:
            deps.append(dep[RosCTypesupportIntrospectionInfo].cc_info)
    deps.append(target[RosCBindingsInfo].cc_info)

    cc_info, dynamic_library = generate_compilation_information(
        ctx = ctx,
        name = "{}__{}__{}__rosidl_typesupport_introspection_c".format(
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
        RosCTypesupportIntrospectionInfo(
            cc_info = cc_info,
            dynamic_libraries = depset(
                direct = [dynamic_library],
                transitive = [
                    dep[RosCTypesupportIntrospectionInfo].dynamic_libraries
                    for dep in ctx.rule.attr.deps
                    if RosCTypesupportIntrospectionInfo in dep
                ],
            ),
            linker_inputs = cc_info.linking_context.linker_inputs,
        ),
    ]

rosidl_typesupport_introspection_c_aspect = aspect(
    implementation = _rosidl_typesupport_introspection_c_aspect_impl,
    toolchains = use_cc_toolchain(),
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        "_c_typesupport_introspection_generator": attr.label(
            default = Label("@rosidl_typesupport_introspection_c//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_c_typesupport_introspection_templates": attr.label(
            default = Label("@rosidl_typesupport_introspection_c//:interface_templates"),
        ),
        "_c_typesupport_introspection_visibility_template": attr.label(
            default = Label("@rosidl_typesupport_introspection_c//:resource/rosidl_typesupport_introspection_c__visibility_control.h.in"),
            allow_single_file = True,
        ),
        "_c_deps": attr.label_list(
            default = [
                Label("@rosidl_typesupport_introspection_c"),
            ],
            providers = [CcInfo],
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo],
        [RosCBindingsInfo],
    ],
    provides = [RosCTypesupportIntrospectionInfo],
)
