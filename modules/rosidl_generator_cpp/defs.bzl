
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
load("@ros//:defs.bzl", "RosInterfaceInfo", "idl_tuple_from_path", "message_info_from_target", "type_description_tuple_from_path")
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_ros_aspect")
load("@rosidl_generator_type_description//:defs.bzl", "RosTypeDescriptionInfo", "type_description_idl_aspect")

def _get_stem(path):
    return path.basename[:-len(path.extension) - 1]

def _generate_source(
    ctx,
    executable,
    mnemonic,
    input_idls,
    input_type_descriptions,
    input_templates,
    templates_hdrs,
    templates_srcs,
    template_visibility_control = None,
    additional = []
):
    # Get information about this package
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)

    # The first output file is the JSON file used as args to the generator.
    input_args = ctx.actions.declare_file(
        "{}/{}_{}_{}.json".format(package_name, message_type, message_name, mnemonic))

    # Prepare hdrs and srcs lists for output.
    output_hdrs = [
        ctx.actions.declare_file(
            "{}/{}/{}".format(package_name, message_type, t.format(message_code))
        ) for t in templates_hdrs
    ]
    output_srcs = [
        ctx.actions.declare_file(
            "{}/{}/{}".format(package_name, message_type, t.format(message_code))
        ) for t in templates_srcs
    ]

    # Write the generator query
    ctx.actions.write(
        input_args,
        json.encode(
            struct(
                package_name = package_name,
                idl_tuples = [idl_tuple_from_path(idl.path) for idl in input_idls],
                output_dir = input_args.dirname,
                template_dir = input_templates[0].dirname,
                type_description_tuples = [type_description_tuple_from_path(idl.path)
                    for idl in input_type_descriptions],
                target_dependencies = [],
            )
        )
    )

    # Pass the query through the generator
    ctx.actions.run(
        inputs = input_idls + input_type_descriptions + input_templates + [input_args],
        outputs = output_hdrs + output_srcs,
        executable = executable,
        arguments = ["--generator-arguments-file={}".format(input_args.path)] + additional,
        mnemonic = mnemonic,
        progress_message = "Running {} for {}".format(mnemonic, ctx.label.name),
    )

    # Optionally include 
    if template_visibility_control:
        output_c_visibility_control_stem = _get_stem(template_visibility_control)
        output_c_visibility_control_h = ctx.actions.declare_file(
            "{}/msg/{}".format(package_name, output_c_visibility_control_stem)
        )
        ctx.actions.expand_template(
            template = template_visibility_control,
            output = output_c_visibility_control_h,
            substitutions = {
                "@PROJECT_NAME@": package_name,
                "@PROJECT_NAME_UPPER@": package_name.upper(),
            },
        )
        output_hdrs.append(output_c_visibility_control_h)

    # Return the headers and sources
    return output_hdrs, output_srcs


def _cc_idl_aspect_impl(target, ctx):
    #print("C_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)

    # Collect all IDLs and JSON files required to generate the language bindings.
    input_idls = [target[RosIdlInfo].idl]
    input_type_descriptions = [target[RosTypeDescriptionInfo].json]
    for dep in ctx.rule.attr.deps:
        if RosIdlInfo in dep:
            input_idls.extend(dep[RosIdlInfo].deps.to_list())
        if RosTypeDescriptionInfo in dep:
            input_type_descriptions.extend(dep[RosTypeDescriptionInfo].deps.to_list())

    # Generate the C++ bindings
    cc_hdrs, cc_srcs = _generate_source(
        ctx = ctx,
        executable = ctx.executable._cc_generator,
        mnemonic = "CcGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._cc_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [
            "{}.hpp",
            "detail/{}__builder.hpp",
            "detail/{}__struct.hpp",
            "detail/{}__traits.hpp",
            "detail/{}__type_support.hpp",
        ],
        templates_srcs = [],
        template_visibility_control = ctx.file._cc_visibility_template,
    )

    # Generate the type support library
    cc_typesupport_hdrs, cc_typesupport_srcs = _generate_source(
        ctx = ctx,
        executable = ctx.executable._cc_typesupport_generator,
        mnemonic = "CcTypeSupportGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._cc_typesupport_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [],
        templates_srcs = ["{}__type_support.cpp"],
        template_visibility_control = None,
        additional = ["--typesupports=rosidl_typesupport_introspection_cpp"]
    )

    # Generate the type support library for introspection
    cc_typesupport_introspection_hdrs, cc_typesupport_introspection_srcs = _generate_source(
        ctx = ctx,
        executable = ctx.executable._cc_typesupport_introspection_generator,
        mnemonic = "CcTypeSupportIntrospectionGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._cc_typesupport_introspection_templates[DefaultInfo].files.to_list(),
        templates_hdrs = ["detail/{}__rosidl_typesupport_introspection_cpp.hpp"],
        templates_srcs = ["detail/{}__type_support.cpp"],
        template_visibility_control = None,
    )

    # Collect the generated source code.
    output_hdrs = cc_hdrs + cc_typesupport_hdrs + cc_typesupport_introspection_hdrs
    output_srcs = cc_srcs + cc_typesupport_srcs + cc_typesupport_introspection_srcs

    # These deps will all have CcInfo providers.
    deps = ctx.attr._cc_deps + ctx.rule.attr.deps

    # Query for the current CC toolchain and feature set.
    cc_toolchain = find_cc_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    # Get a compilation context.
    compilation_context, compilation_outputs = cc_common.compile(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = output_srcs,
        public_hdrs = output_hdrs,
        compilation_contexts = [
            dep[CcInfo].compilation_context for dep in deps if CcInfo in dep
        ]
    )

    # Get a linking context.
    if output_srcs:
        linking_context, _ = cc_common.create_linking_context_from_compilation_outputs(
            name = ctx.label.name,
            actions = ctx.actions,
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
            compilation_outputs = compilation_outputs,
            linking_contexts = [
                dep[CcInfo].linking_context for dep in deps if CcInfo in dep
            ],
        )
    else:
        linking_context = cc_common.merge_linking_contexts(
            linking_contexts = [
                dep[CcInfo].linking_context for dep in deps if CcInfo in dep
            ]
        )

    # Return a CcInfo provider for the aspect.
    return [
        CcInfo(
            compilation_context = compilation_context,
            linking_context = linking_context,
        )
    ]

cc_idl_aspect = aspect(
    implementation = _cc_idl_aspect_impl,
    toolchains = use_cc_toolchain(),
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
        [RosTypeDescriptionInfo]
    ],
    provides = [CcInfo],
)

def _cc_ros_library_impl(ctx):
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [
            dep[CcInfo] for dep in ctx.attr.deps
        ],
    )
    return [cc_info]

cc_ros_library = rule(
    implementation = _cc_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_ros_aspect,              # RosIdlInfo <- RosInterfaceInfo
                type_description_idl_aspect, # RosTypeDescriptionInfo <- RosIdlInfo
                cc_idl_aspect,               # CcInfo <- {RosIdlInfo, RosTypeDescriptionInfo}
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [CcInfo],
)

