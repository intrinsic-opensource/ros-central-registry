
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
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain", "use_cc_toolchain")
load("@ros//:defs.bzl", "RosInterfaceInfo")
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_ros_aspect", "generate_sources", "generate_cc_info")
load("@rosidl_generator_type_description//:defs.bzl", "RosTypeDescriptionInfo", "type_description_idl_aspect")

RosCBindingsInfo = provider(
    "Encapsulates C information generated for an underlying ROS message.", 
    fields = [
        "cc_info",
    ]
)
def _c_idl_aspect_impl(target, ctx):
    #print("C_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)

    # Collect all IDLs and JSON files required to generate the language bindings.
    input_idls = target[RosIdlInfo].idls.to_list()
    input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list()

    # Generate the C bindings
    c_hdrs, c_srcs, c_include_dir = generate_sources(
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
        ],
        template_visibility_control = ctx.file._c_visibility_template,
    )

    # Generate the type support library
    c_typesupport_hdrs, c_typesupport_srcs, _ = generate_sources(
        ctx = ctx,
        executable = ctx.executable._c_typesupport_generator,
        mnemonic = "CTypeSupportGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._c_typesupport_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [],
        templates_srcs = ["{}__type_support.c"],
        template_visibility_control = None,
        additional = ["--typesupports=rosidl_typesupport_introspection_c"]
    )

    # Generate the type support library for introspection
    c_typesupport_introspection_hdrs, c_typesupport_introspection_srcs, _ = generate_sources(
        ctx = ctx,
        executable = ctx.executable._c_typesupport_introspection_generator,
        mnemonic = "CTypeSupportIntrospectionGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._c_typesupport_introspection_templates[DefaultInfo].files.to_list(),
        templates_hdrs = ["detail/{}__rosidl_typesupport_introspection_c.h"],
        templates_srcs = ["detail/{}__type_support.c"],
        template_visibility_control = ctx.file._c_typesupport_introspection_visibility_template,
    )

    # These deps will all have CcInfo providers.
    cc_info_deps = [dep[CcInfo] for dep in ctx.attr._c_deps if CcInfo in dep]
    for dep in ctx.rule.attr.deps:
        if RosCBindingsInfo in dep:
            cc_info_deps.extend([d for d in dep[RosCBindingsInfo].cc_info.to_list()])
    
    # Merge headers, sources and deps into a CcInfo provider.
    cc_info = generate_cc_info(
        ctx = ctx,
        name = "{}_c".format(ctx.label.name),
        hdrs = c_hdrs + c_typesupport_hdrs + c_typesupport_introspection_hdrs,
        srcs = c_srcs + c_typesupport_srcs + c_typesupport_introspection_srcs,
        include_dirs = [c_include_dir],
        deps = cc_info_deps,
    )

    # Return a CcInfo provider for the aspect.
    return [
        RosCBindingsInfo(
            cc_info = depset(
                direct = [cc_info],
                transitive = [
                    dep[RosCBindingsInfo].cc_info
                        for dep in ctx.rule.attr.deps if RosCBindingsInfo in dep
                ],
            )
        )
    ]

c_idl_aspect = aspect(
    implementation = _c_idl_aspect_impl,
    toolchains = use_cc_toolchain(),
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        #########################################################################
        # Code generation #######################################################
        #########################################################################
        
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

        #########################################################################
        # Type support generation ###############################################
        #########################################################################
        
        "_c_typesupport_generator": attr.label(
            default = Label("@rosidl_typesupport_c//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_c_typesupport_templates": attr.label(
            default = Label("@rosidl_typesupport_c//:interface_templates"),
        ),

        #########################################################################
        # Introspection type support generation #################################
        #########################################################################
        
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

        #########################################################################
        # Dependencies ##########################################################
        #########################################################################
        
        "_c_deps": attr.label_list(
            default = [
                Label("@rosidl_runtime_c"),
                Label("@rosidl_typesupport_c"),
                Label("@rosidl_typesupport_introspection_c"),
            ],
            providers = [CcInfo],
        ),
        
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo],
    ],
    provides = [RosCBindingsInfo],
)

def _c_ros_library_impl(ctx):
    direct_cc_infos = []
    for dep in ctx.attr.deps:
        direct_cc_infos.extend(dep[RosCBindingsInfo].cc_info.to_list())
    return [
        cc_common.merge_cc_infos(
            direct_cc_infos = direct_cc_infos
        )
    ]

c_ros_library = rule(
    implementation = _c_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_ros_aspect,              # RosIdlInfo <- RosInterfaceInfo
                type_description_idl_aspect, # RosTypeDescriptionInfo <- RosIdlInfo
                c_idl_aspect,                # CcInfo <- {RosIdlInfo, RosTypeDescriptionInfo}
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [CcInfo],
)

