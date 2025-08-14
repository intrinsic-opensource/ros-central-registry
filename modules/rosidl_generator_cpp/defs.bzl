
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

load("@rules_cc//cc:defs.bzl", "CcInfo", "cc_common")
load("@ros//:defs.bzl", "RosInterfaceInfo")
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_aspect", "generate_sources", "generate_cc_info")
load("@rosidl_generator_c//:defs.bzl", "RosCBindingsInfo", "c_aspect")
load("@rosidl_generator_type_description//:defs.bzl", "RosTypeDescriptionInfo", "type_description_aspect")

RosCcBindingsInfo = provider(
    "Encapsulates C++ information generated for an underlying ROS message.", 
    fields = [
        "cc_infos",
        "cc_files",
    ]
)

def _cc_aspect_impl(target, ctx):
    #print("C_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)

    # Collect all IDLs and JSON files required to generate the language bindings.
    input_idls = target[RosIdlInfo].idls.to_list()
    input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list()

    # Generate the C++ bindings
    cc_hdrs, cc_srcs, cc_include_dir = generate_sources(
        ctx = ctx,
        executable = ctx.executable._cc_generator,
        mnemonic = "CcGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._cc_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [
            "{package_name}/{message_type}/{message_code}.hpp",
            "{package_name}/{message_type}/detail/{message_code}__builder.hpp",
            "{package_name}/{message_type}/detail/{message_code}__struct.hpp",
            "{package_name}/{message_type}/detail/{message_code}__traits.hpp",
            "{package_name}/{message_type}/detail/{message_code}__type_support.hpp",
        ],
        templates_srcs = [],
        template_visibility_control = ctx.file._cc_visibility_template,
    )

    # Generate the type support library
    cc_typesupport_hdrs, cc_typesupport_srcs, _ = generate_sources(
        ctx = ctx,
        executable = ctx.executable._cc_typesupport_generator,
        mnemonic = "CcTypeSupportGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._cc_typesupport_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [],
        templates_srcs = ["{package_name}/{message_type}/{message_code}__type_support.cpp"],
        template_visibility_control = None,
        additional = ["--typesupports=rosidl_typesupport_introspection_cpp"]
    )

    # Generate the type support library for introspection
    cc_typesupport_introspection_hdrs, cc_typesupport_introspection_srcs, _ = generate_sources(
        ctx = ctx,
        executable = ctx.executable._cc_typesupport_introspection_generator,
        mnemonic = "CcTypeSupportIntrospectionGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._cc_typesupport_introspection_templates[DefaultInfo].files.to_list(),
        templates_hdrs = ["{package_name}/{message_type}/detail/{message_code}__rosidl_typesupport_introspection_cpp.hpp"],
        templates_srcs = ["{package_name}/{message_type}/detail/{message_code}__type_support.cpp"],
        template_visibility_control = None,
    )

    # These deps will all have CcInfo providers. We need to combine the library
    # dependencies with the C generated headers and C++ generated headers.
    deps = [dep[CcInfo] for dep in ctx.attr._cc_deps if CcInfo in dep] 
    deps.extend([d for d in target[RosCBindingsInfo].cc_infos.to_list()])
    for dep in ctx.rule.attr.deps:
        if RosCcBindingsInfo in dep:
            deps.extend([d for d in dep[RosCcBindingsInfo].cc_infos.to_list()])

    # Merge headers, sources and deps into a CcInfo provider.
    hdrs = cc_hdrs + cc_typesupport_hdrs + cc_typesupport_introspection_hdrs
    srcs = cc_srcs + cc_typesupport_srcs + cc_typesupport_introspection_srcs
    cc_info, _, _ = generate_cc_info(
        ctx = ctx,
        name = "{}_cc".format(ctx.label.name),
        hdrs = hdrs,
        srcs = srcs,
        include_dirs = [cc_include_dir],
        deps = deps,
    )

    # Return a CcInfo provider for the aspect.
    return [
        RosCcBindingsInfo(
            cc_infos = depset(
                direct = [cc_info],
                transitive = [
                    dep[RosCcBindingsInfo].cc_infos
                        for dep in ctx.rule.attr.deps if RosCcBindingsInfo in dep
                ],
            ),
            cc_files = depset(
                direct = hdrs + srcs,
                transitive = [
                    dep[RosCcBindingsInfo].cc_files
                        for dep in ctx.rule.attr.deps if RosCcBindingsInfo in dep
                ],
            ),
        )
    ]

cc_aspect = aspect(
    implementation = _cc_aspect_impl,
    toolchains = ["@rules_cc//cc:toolchain_type"],
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        #########################################################################
        # Code generation #######################################################
        #########################################################################
        
        "_cc_generator": attr.label(
            default = Label("//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_cc_templates": attr.label(
            default = Label("//:interface_templates"),
        ),
        "_cc_visibility_template": attr.label(
            default = Label("//:resource/rosidl_generator_cpp__visibility_control.hpp.in"),
            allow_single_file = True,
        ),

        #########################################################################
        # Type support generation ###############################################
        #########################################################################
        
        "_cc_typesupport_generator": attr.label(
            default = Label("@rosidl_typesupport_cpp//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_cc_typesupport_templates": attr.label(
            default = Label("@rosidl_typesupport_cpp//:interface_templates"),
        ),

        #########################################################################
        # Introspection type support generation #################################
        #########################################################################
        
        "_cc_typesupport_introspection_generator": attr.label(
            default = Label("@rosidl_typesupport_introspection_cpp//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_cc_typesupport_introspection_templates": attr.label(
            default = Label("@rosidl_typesupport_introspection_cpp//:interface_templates"),
        ),

        #########################################################################
        # Dependencies ##########################################################
        #########################################################################
        
        "_cc_deps": attr.label_list(
            default = [
                Label("@rosidl_runtime_cpp"),
                Label("@rosidl_typesupport_cpp"),
                Label("@rosidl_typesupport_introspection_cpp"),
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
    provides = [RosCcBindingsInfo],
)

def _cc_ros_library_impl(ctx):
    cc_infos = []
    for dep in ctx.attr.deps:
        if RosCBindingsInfo in dep:
            cc_infos.extend(dep[RosCBindingsInfo].cc_infos.to_list())
        if RosCcBindingsInfo in dep:
            cc_infos.extend(dep[RosCcBindingsInfo].cc_infos.to_list())
    return [
        cc_common.merge_cc_infos(direct_cc_infos = cc_infos), # <--- CcInfo
        DefaultInfo(
            files = depset(
                transitive = [
                    dep[RosCBindingsInfo].cc_files
                        for dep in ctx.attr.deps if RosCBindingsInfo in dep
                ] + [
                    dep[RosCcBindingsInfo].cc_files
                        for dep in ctx.attr.deps if RosCcBindingsInfo in dep
                ]
            )
        ),
    ]

cc_ros_library = rule(
    implementation = _cc_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_aspect,              # RosIdlInfo <- RosInterfaceInfo
                type_description_aspect, # RosTypeDescriptionInfo <- RosIdlInfo
                c_aspect,                # RosCBindingsInfo <- {RosIdlInfo, RosTypeDescriptionInfo}
                cc_aspect,               # RosCcBindingsInfo <- {RosIdlInfo, RosTypeDescriptionInfo}
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [CcInfo, DefaultInfo],
)

