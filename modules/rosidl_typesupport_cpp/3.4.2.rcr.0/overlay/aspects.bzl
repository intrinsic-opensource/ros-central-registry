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

load("@rosidl_generator_cpp//:types.bzl", "RosCcBindingsInfo")
load("@rosidl_generator_type_description//:types.bzl", "RosTypeDescriptionInfo")
load("@rosidl_pycommon//:defs.bzl", "RosIdlInfo", "RosInterfaceInfo", "generate_compilation_information", "generate_sources")
load("@rules_cc//cc:defs.bzl", "CcInfo")
load("@rules_cc//cc:find_cc_toolchain.bzl", "use_cc_toolchain")
load(":types.bzl", "RosCcTypesupportInfo")

def _rosidl_typesupport_cpp_aspect_impl(target, ctx):
    hdrs, srcs, include_dirs = generate_sources(
        target = target,
        ctx = ctx,
        executable = ctx.executable._cc_typesupport_generator,
        mnemonic = "CcTypeSupportGeneration",
        input_idls = [target[RosIdlInfo].idl],
        input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list(),
        input_templates = ctx.attr._cc_typesupport_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [],
        templates_srcs = ["detail/{}__rosidl_typesupport_cpp.cpp"],
        additional = [
            "--typesupports",
            "rosidl_typesupport_fastrtps_cpp",
            "rosidl_typesupport_introspection_cpp",
        ],
    )

    deps = [dep[CcInfo] for dep in ctx.attr._cc_deps if CcInfo in dep]
    deps.append(target[RosCcBindingsInfo].cc_info)
    for dep in ctx.rule.attr.deps:
        if RosCcTypesupportInfo in dep:
            deps.append(dep[RosCcTypesupportInfo].cc_info)

    cc_info, dynamic_libraries = generate_compilation_information(
        ctx = ctx,
        name = "{}__{}__{}__rosidl_typesupport_cpp".format(
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
        RosCcTypesupportInfo(
            cc_info = cc_info,
            dynamic_libraries = depset(
                direct = dynamic_libraries,
                transitive = [
                    dep[RosCcTypesupportInfo].dynamic_libraries
                    for dep in ctx.rule.attr.deps
                    if RosCcTypesupportInfo in dep
                ],
            ),
        ),
    ]

rosidl_typesupport_cpp_aspect = aspect(
    implementation = _rosidl_typesupport_cpp_aspect_impl,
    toolchains = use_cc_toolchain(),
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        "_cc_typesupport_generator": attr.label(
            default = Label("@rosidl_typesupport_cpp//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_cc_typesupport_templates": attr.label(
            default = Label("@rosidl_typesupport_cpp//:interface_templates"),
        ),
        "_cc_deps": attr.label_list(
            default = [
                Label("@rosidl_typesupport_cpp//:rosidl_typesupport_cpp_library"),
            ],
            providers = [CcInfo],
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo],
        [RosCcBindingsInfo],
    ],
    provides = [RosCcTypesupportInfo],
)
