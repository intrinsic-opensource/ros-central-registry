
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

load("@protobuf//bazel/common:proto_common.bzl", "proto_common")
load("@protobuf//bazel/common:proto_info.bzl", "ProtoInfo")
load("@rules_cc//cc:find_cc_toolchain.bzl", "use_cc_toolchain")
load("@ros//:defs.bzl", "RosInterfaceInfo")
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_aspect",
    "message_info_from_target", "idl_tuple_from_path", "generate_sources", "generate_cc_info")

def _proto_aspect_impl(target, ctx):
    #print("TYPE_DESCRIPTION_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)

    # These are all the bits and pieces produced by this aspect to generate ProtoInfo and CcInfo.
    output_proto = ctx.actions.declare_file(
        "{}/{}/{}.proto".format(package_name, message_type, message_name))
    output_descriptor = ctx.actions.declare_file(
        "{}/{}/{}-descriptor-set.proto.bin".format(package_name, message_type, message_name))
    # output_proto_h = ctx.actions.declare_file(
    #     "{}/{}/{}.pb.h".format(package_name, message_type, message_name))
    # output_proto_cc = ctx.actions.declare_file(
    #     "{}/{}/{}.pb.cc".format(package_name, message_type, message_name))

    # The .proto file will end up at this path: <base...>/<package_name>/msg/Name.proto. Since imports
    # are all relative in the proto world, we must calculate both the root and relative file names:
    #   proto_source_root = <base...>
    #   proto_source_file = <package_name>/msg/Name.proto
    #proto_source_root = "/".join(output_proto.short_path.split("/")[:-3])
    #proto_source_file = "/".join(output_proto.short_path.split("/")[-3:])

    # Call the idl_adapter_proto python tool to transform the IDL collection into a proto collection.
    proto_hdrs, proto_srcs, proto_include_dir = generate_sources(
        ctx = ctx,
        executable = ctx.executable._proto_generator,
        mnemonic = "IdlToProtobuf",
        input_idls = target[RosIdlInfo].idls.to_list(),
        input_type_descriptions = [],
        input_templates = ctx.attr._proto_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [],
        templates_srcs = ["{}.proto"],
        template_visibility_control = ctx.file._proto_visibility_template,
        message_is_pascal_case = False,
    )
    
    # Construct the ProtoInfo providers.
    proto_info = ProtoInfo(
        srcs = [output_proto],
        deps = [dep[ProtoInfo] for dep in ctx.rule.attr.deps if ProtoInfo in dep],
        descriptor_set = output_descriptor,
        proto_path = "",
        workspace_root = ctx.label.workspace_root,
    )

    # Generate the descriptor_set for the provider.
    additional_args = ctx.actions.args()
    additional_args.add("--retain_options")
    proto_common.compile(
        ctx.actions,
        proto_info,
        ctx.toolchains["@protobuf//bazel/private:python_toolchain_type"].proto,
        generated_files = [output_descriptor],
        additional_inputs = depset(
            transitive = [
                dep[ProtoInfo].transitive_descriptor_sets
                    for dep in ctx.rule.attr.deps if ProtoInfo in dep
            ]
        ),
        additional_args = additional_args
    )

    # Collect all of the sources from the dependencies.
    return [
        proto_info
    ]

proto_aspect = aspect(
    implementation = _proto_aspect_impl,
    attr_aspects = ["deps"],
    fragments = ["proto"],
    toolchains = ["@protobuf//bazel/private:python_toolchain_type"],
    attrs = {
        "_proto_generator": attr.label(
            default = Label("//:cli"),
            executable = True,
            cfg = "exec",
        ),        
        "_proto_templates": attr.label(
            default = Label("//:interface_templates"),
        ),
        "_proto_visibility_template": attr.label(
            default = Label("//:resource/rosidl_adapter_proto__visibility_control.h.in"),
            allow_single_file = True,
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [RosIdlInfo],
    provides = [ProtoInfo],
)

def _proto_ros_library_impl(ctx):
    # EEK: How would you merge multiple ProtoInfo providers? This is the only way I can think of
    # doing it, but it definitely doesn't feel right, or even work for that matter.
    return [
        ProtoInfo(
            srcs = [],
            deps = [dep[ProtoInfo] for dep in ctx.attr.deps if ProtoInfo in dep],
            descriptor_set = ctx.attr.deps[0][ProtoInfo].direct_descriptor_set,
            proto_path = "",
            workspace_root = ctx.label.workspace_root,
        )
    ]

proto_ros_library = rule(
    implementation = _proto_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_aspect,
                proto_aspect,
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [ProtoInfo],
)

