
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

C_GENERATOR_TEMPLATES_HDRS = [
    "{}.h",
    "detail/{}__functions.h",
    "detail/{}__struct.h",
    "detail/{}__type_support.h",
]

C_GENERATOR_TEMPLATES_SRCS = [
    "detail/{}__description.c",
    "detail/{}__functions.c",
    "detail/{}__type_support.c",
]

def _c_idl_aspect_impl(target, ctx):
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

    # Collect inputs
    input_templates = ctx.attr._interface_templates[DefaultInfo].files.to_list()

    # Collect tuples
    idl_tuples = [idl_tuple_from_path(idl.path) for idl in input_idls]
    type_description_tuples = [type_description_tuple_from_path(idl.path)
        for idl in input_type_descriptions]

    # The first output file is the JSON file used as args to the generator.
    input_args = ctx.actions.declare_file(
        "{}/{}_{}_c.json".format(package_name, message_type, message_name))
    ctx.actions.write(
        input_args,
        json.encode(
            struct(
                package_name = package_name,
                idl_tuples = idl_tuples,
                output_dir = input_args.dirname,
                template_dir = input_templates[0].dirname,
                type_description_tuples = type_description_tuples,
                target_dependencies = [],
            )
        )
    )
    
    # Iterate over all the IDLs included in the bundle required for this
    # target, and make sure we have declared output files for each one.
    output_hdrs = []
    output_srcs = []
    for idl in input_idls:
        for template in C_GENERATOR_TEMPLATES_HDRS:
            generated_file = "{}/{}/{}".format(
                package_name,
                message_type,
                template.format(message_code),
            )
            output_hdrs.append(ctx.actions.declare_file(generated_file))
        for template in C_GENERATOR_TEMPLATES_SRCS:
            generated_file = "{}/{}/{}".format(
                package_name,
                message_type,
                template.format(message_code),
            )
            output_srcs.append(ctx.actions.declare_file(generated_file))

    # Run the action to generate the files
    ctx.actions.run(
        inputs = input_idls + input_type_descriptions + input_templates + [input_args],
        outputs = output_hdrs + output_srcs,
        executable = ctx.executable._rosidl_generator,
        arguments = [
            "--generator-arguments-file={}".format(input_args.path),
        ],
        mnemonic = "IdlAndTypeDescriptionToC",
        progress_message = "Generating C files for {}".format(ctx.label.name),
    )

    # Generate a visibility header for packages.
    visibility_control_h = ctx.actions.declare_file(
        "{}/msg/rosidl_generator_c__visibility_control.h".format(package_name)
    )
    ctx.actions.expand_template(
        template = ctx.attr._visibility_control_template[DefaultInfo].files.to_list()[0],
        output = visibility_control_h,
        substitutions = {
            "@PROJECT_NAME@": package_name,
            "@PROJECT_NAME_UPPER@": package_name.upper(),
        },
    )
    output_hdrs.append(visibility_control_h)

    # Collect all cc deps
    deps = ctx.rule.attr.deps + [
        ctx.attr._rosidl_runtime_c,
        ctx.attr._rosidl_typesupport_interface
    ]

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
    libraries = []
    if output_srcs:
        linking_context, linking_outputs = cc_common.create_linking_context_from_compilation_outputs(
            name = ctx.label.name,
            actions = ctx.actions,
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
            compilation_outputs = compilation_outputs,
            linking_contexts = [
                dep[CcInfo].linking_context for dep in deps if CcInfo in dep
            ],
        )
        library_to_link = linking_outputs.library_to_link
        if library_to_link and library_to_link.static_library:
            libraries.append(library_to_link.static_library)
        if library_to_link and library_to_link.pic_static_library:
            libraries.append(library_to_link.pic_static_library)
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

c_idl_aspect = aspect(
    implementation = _c_idl_aspect_impl,
    toolchains = use_cc_toolchain(),
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        "_rosidl_generator": attr.label(
            default = Label("//:rosidl_generator"),
            executable = True,
            cfg = "exec",
        ),
        "_interface_templates": attr.label(
            default = Label("//:interface_templates"),
        ),
        "_visibility_control_template": attr.label(
            default = Label("//:visibility_control_template"),
            allow_single_file = True,
        ),
        "_rosidl_runtime_c": attr.label(
            default = Label("@rosidl_runtime_c"),
        ),
        "_rosidl_typesupport_interface": attr.label(
            default = Label("@rosidl_typesupport_interface"),
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo]
    ],
    provides = [CcInfo],
)

def _c_ros_library_impl(ctx):
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [
            dep[CcInfo] for dep in ctx.attr.deps
        ],
    )
    return [cc_info]

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

