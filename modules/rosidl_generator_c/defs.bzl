
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
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_aspect", "generate_sources", "generate_cc_info")
load("@rosidl_adapter_proto//:defs.bzl", "proto_aspect")
load("@rosidl_generator_type_description//:defs.bzl", "RosTypeDescriptionInfo", "type_description_aspect")

RosCBindingsInfo = provider(
    "Encapsulates C information generated for an underlying ROS message.", 
    fields = [
        "cc_infos",
        "cc_files",
    ]
)
def _c_aspect_impl(target, ctx):
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

    # Generate the type support library for fastrtps
    c_typesupport_fastrtps_hdrs, c_typesupport_fastrtps_srcs, _ = generate_sources(
        ctx = ctx,
        executable = ctx.executable._c_typesupport_fastrtps_generator,
        mnemonic = "CTypeSupportFastRTPSGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._c_typesupport_fastrtps_templates[DefaultInfo].files.to_list(),
        templates_hdrs = ["detail/{}__rosidl_typesupport_fastrtps_c.h"],
        templates_srcs = ["detail/{}__type_support_c.cpp"],
        template_visibility_control = ctx.file._c_typesupport_fastrtps_visibility_template,
    )

    # Generate the type support library for protobuf
    c_typesupport_protobuf_hdrs, c_typesupport_protobuf_srcs, _ = generate_sources(
        ctx = ctx,
        executable = ctx.executable._c_typesupport_protobuf_generator,
        mnemonic = "CTypeSupportProtobufGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._c_typesupport_protobuf_templates[DefaultInfo].files.to_list(),
        templates_hdrs = ["{}__rosidl_typesupport_protobuf_c.hpp"],
        templates_srcs = ["detail/protobuf_c/{}__type_support.cpp"],
        template_visibility_control = ctx.file._c_typesupport_protobuf_visibility_template,
    )

    # These deps will all have CcInfo providers.
    deps = [dep[CcInfo] for dep in ctx.attr._c_deps if CcInfo in dep]
    deps.append(target[CcInfo])
    for dep in ctx.rule.attr.deps:
        if CcInfo in dep:
            deps.append(dep[CcInfo])
        if RosCBindingsInfo in dep:
            deps.extend([d for d in dep[RosCBindingsInfo].cc_infos.to_list()])
    
    # Merge headers, sources and deps into a CcInfo provider.
    hdrs = c_hdrs + c_typesupport_introspection_hdrs + c_typesupport_fastrtps_hdrs + c_typesupport_protobuf_hdrs
    srcs = c_srcs + c_typesupport_introspection_srcs + c_typesupport_fastrtps_srcs + c_typesupport_protobuf_srcs
    cc_info = generate_cc_info(
        ctx = ctx,
        name = "{}_c".format(ctx.label.name),
        hdrs = hdrs,
        srcs = srcs,
        include_dirs = [c_include_dir],
        deps = deps,
    )

    # Return a CcInfo provider for the aspect.
    return [
        RosCBindingsInfo(
            cc_infos = depset(
                direct = [cc_info],
                transitive = [
                    dep[RosCBindingsInfo].cc_infos
                        for dep in ctx.rule.attr.deps if RosCBindingsInfo in dep
                ],
            ),
            cc_files = depset(
                direct = hdrs + srcs,
                transitive = [
                    dep[RosCBindingsInfo].cc_files
                        for dep in ctx.rule.attr.deps if RosCBindingsInfo in dep
                ],
            ),
        )
    ]

c_aspect = aspect(
    implementation = _c_aspect_impl,
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
        # FastRTPS type support generation ######################################
        #########################################################################
        
        "_c_typesupport_fastrtps_generator": attr.label(
            default = Label("@rosidl_typesupport_fastrtps_c//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_c_typesupport_fastrtps_templates": attr.label(
            default = Label("@rosidl_typesupport_fastrtps_c//:interface_templates"),
        ),
        "_c_typesupport_fastrtps_visibility_template": attr.label(
            default = Label("@rosidl_typesupport_fastrtps_c//:resource/rosidl_typesupport_fastrtps_c__visibility_control.h.in"),
            allow_single_file = True,
        ),

        #########################################################################
        # Protobuf type support generation ######################################
        #########################################################################
        
        "_c_typesupport_protobuf_generator": attr.label(
            default = Label("@rosidl_typesupport_protobuf_c//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_c_typesupport_protobuf_templates": attr.label(
            default = Label("@rosidl_typesupport_protobuf_c//:interface_templates"),
        ),
        "_c_typesupport_protobuf_visibility_template": attr.label(
            default = Label("@rosidl_typesupport_protobuf_c//:resource/rosidl_typesupport_protobuf_c__visibility_control.h.in"),
            allow_single_file = True,
        ),

        #########################################################################
        # Dependencies ##########################################################
        #########################################################################
        
        "_c_deps": attr.label_list(
            default = [
                Label("@rosidl_runtime_c"),
                Label("@rosidl_typesupport_fastrtps_c"),
                Label("@rosidl_typesupport_protobuf_c"),
                Label("@rosidl_typesupport_introspection_c"),
                Label("@rclcpp//:type_adapter"),
                Label("@rmw"),        
            ],
            providers = [CcInfo],
        ),
        
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo],
        [CcInfo]
    ],
    provides = [RosCBindingsInfo],
)

def _c_ros_library_impl(ctx):
    cc_infos = []
    for dep in ctx.attr.deps:
        cc_infos.extend(dep[RosCBindingsInfo].cc_infos.to_list())
    return [
        cc_common.merge_cc_infos(direct_cc_infos = cc_infos), # <--- CcInfo
        DefaultInfo(
            files = depset(
                transitive = [
                    dep[RosCBindingsInfo].cc_files
                        for dep in ctx.attr.deps if RosCBindingsInfo in dep
                ]
            )
        ),
    ]

c_ros_library = rule(
    implementation = _c_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_aspect,              # RosIdlInfo <- RosInterfaceInfo
                proto_aspect,            # {ProtoInfo, CcInfo} <- RosIdlInfo
                type_description_aspect, # RosTypeDescriptionInfo <- RosIdlInfo
                c_aspect,                # RosCBindingsInfo <- {RosIdlInfo, CcInfo, RosTypeDescriptionInfo}
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [CcInfo, DefaultInfo],
)

