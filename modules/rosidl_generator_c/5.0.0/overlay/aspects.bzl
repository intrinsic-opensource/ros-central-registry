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

load("@rosidl_adapter//:tools.bzl", "generate_cc_info", "generate_sources")
load("@rosidl_adapter//:types.bzl", "RosIdlInfo")
load("@rosidl_cmake//:types.bzl", "RosInterfaceInfo")
load("@rosidl_generator_type_description//:types.bzl", "RosTypeDescriptionInfo")
load("@rules_cc//cc:defs.bzl", "CcInfo", "cc_common")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain", "use_cc_toolchain")
load(":types.bzl", "RosCBindingsInfo", "RosCBindingsFilesInfo")

def _c_files_aspect_impl(target, ctx):
    input_idls = target[RosIdlInfo].idls.to_list()
    input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list()

    # Generate the C bindings
    hdrs, srcs, include_dir = generate_sources(
        target = target,
        ctx = ctx,
        executable = ctx.executable._c_generator,
        mnemonic = "CGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._c_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [
            "{}.h",
            "detail/{}__functions.h",
            "detail/{}__struct.h",
            "detail/{}__type_support.h",
        ],
        templates_srcs = [
            "detail/{}__description.c",
            "detail/{}__functions.c",
            "detail/{}__type_support.c",
        ],
        template_visibility_control = ctx.file._c_visibility_template,
    )

    return [
        RosCBindingsFilesInfo(
            hdrs = hdrs,
            srcs = srcs,
            include_dirs = [include_dir],
        ),
    ]

c_files_aspect = aspect(
    implementation = _c_files_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_c_generator": attr.label(
            default = Label("//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_c_templates": attr.label(
            default = Label("//:interface_templates"),
        ),
        "_c_visibility_template": attr.label(
            default = Label("//:resource/rosidl_generator_c__visibility_control.h.in"),
            allow_single_file = True,
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo],
    ],
    provides = [RosCBindingsFilesInfo],
)

# Aspect to generate the C bindings and provide CcInfo for them.

def _c_aspect_impl(target, ctx):

    # Collect dependencies
    deps = [dep[CcInfo] for dep in ctx.attr._c_deps if CcInfo in dep]
    for dep in ctx.rule.attr.deps:
        if RosCBindingsInfo in dep:
            deps.append(dep[RosCBindingsInfo].cc_info)

    # Assemble the CcInfo provider.
    cc_info = generate_cc_info(
        ctx = ctx,
        name = "{}_c".format(ctx.label.name),
        hdrs = target[RosCBindingsFilesInfo].hdrs,
        srcs = target[RosCBindingsFilesInfo].srcs,
        deps = deps,
        include_dirs = target[RosCBindingsFilesInfo].include_dirs,
    )

    # Return the CcInfo wrapped in a RosCBindingsInfo provider.
    return [
        RosCBindingsInfo(
            cc_info = cc_info
        ),
    ]

c_aspect = aspect(
    implementation = _c_aspect_impl,
    toolchains = use_cc_toolchain(),
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        "_c_deps": attr.label_list(
            default = [
                Label("@rosidl_runtime_c"),
            ],
            providers = [CcInfo],
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosCBindingsFilesInfo],
    ],
    provides = [RosCBindingsInfo],
)
