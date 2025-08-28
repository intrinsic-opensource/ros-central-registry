
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

RosProtoInfo = provider(
    "Encapsulates protobuf information generated for an underlying IDL.", 
    fields = [
        "protos",
    ]
)

def _flatten_proto_info(ctx, proto, descriptor_set, deps, workspace_root, proto_path):    
    # Use the depset data structure to deduplicate the proto_infos from the whole dep tree.
    protos = depset(
        transitive = [
            dep[RosProtoInfo].protos for dep in deps if RosProtoInfo in dep
        ]
    )

    # Rewrite all srcs to be relative to the current workspace_root. This is a necessary step in
    # order for the `ProtoInfo` to work with the `cc_proto_aspect` to generate a CcInfo provider.
    virtual_srcs = []
    for dep_proto in protos.to_list():
        virtual_proto = ctx.actions.declare_file(
            dep_proto.short_path.replace("..", "external").replace(dep_proto.owner.workspace_root, proto_path))
        ctx.actions.symlink(output = virtual_proto, target_file = dep_proto)
        virtual_srcs.append(virtual_proto)
    
    # Return a new list of ProtoInfo with direct sources.
    return ProtoInfo(
        srcs = virtual_srcs,                        # eg. external/builtin_interfaces+/virtual_imports/msg__Time/builtin_interfaces/msg/Time.proto
        descriptor_set = descriptor_set,            # eg. external/builtin_interfaces+/virtual_imports/msg__Time/builtin_interfaces/msg/Time.descriptor-set.bin
        workspace_root = ctx.label.workspace_root,  # eg. external/builtin_interfaces+
        proto_path = proto_path,                    # eg. _virtual_imports/msg__Time
        bin_dir = ctx.bin_dir.path,                 # Required for virtual paths
        deps = [],                                  # Empty, because we are intentionally flattening.
    )

def _proto_aspect_impl(target, ctx):
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)
    target_name = target.label.name

    # These are all the bits and pieces produced by this aspect to generate ProtoInfo and CcInfo.
    proto = ctx.actions.declare_file(
        "{}/{}/{}.proto".format(package_name, message_type, message_name))
    descriptor_set = ctx.actions.declare_file(
        "{}/{}/{}.bin".format(package_name, message_type, message_name))

    # We are going to use a target-name prefixed workspace to avoid symlink collisions.
    proto_path = "_virtual_imports/{}".format(target_name)

    # Call the idl_adapter_proto python tool to transform the IDL collection into a proto collection.
    _, protos, proto_include_dir = generate_sources(
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

    # Flatten our depset of protos into direct source. Not efficient, but it works.
    proto_info = _flatten_proto_info(
        ctx = ctx,
        proto = proto,
        descriptor_set = descriptor_set,
        deps = ctx.rule.attr.deps,
        workspace_root = ctx.label.workspace_root,
        proto_path = proto_path,
    )

    # Create the descriptor for this specific proto.
    proto_toolchain = ctx.toolchains["@protobuf//bazel/private:proto_toolchain_type"]
    proto_common.compile(
        actions = ctx.actions,
        proto_info = proto_info,
        proto_lang_toolchain_info = proto_toolchain.proto,
        generated_files = [descriptor_set],
    )

    # Return the RosProtoInfo provider.
    return [
        RosProtoInfo(
            protos = depset(
                direct = [proto],
                transitive = [
                    dep[RosProtoInfo].protos
                        for dep in ctx.rule.attr.deps if RosProtoInfo in dep
                ],
            ),
        ),
        proto_info
    ]

proto_aspect = aspect(
    implementation = _proto_aspect_impl,
    attr_aspects = ["deps"],
    fragments = ["proto"],
    toolchains = ["@protobuf//bazel/private:proto_toolchain_type"],
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
    provides = [RosProtoInfo, ProtoInfo],
)

def _proto_ros_library_impl(ctx):
    # We are going to use a target-name prefixed workspace to avoid symlink collisions.
    proto_path = "_virtual_imports/{}".format(ctx.label.name)

    # Since there are no direct srcs, the content of the descriptor set does not matter.
    virtual_proto = ctx.actions.declare_file(proto_path + "/dummy.proto")
    ctx.actions.write(virtual_proto, "")
    virtual_descriptor_set = ctx.actions.declare_file(proto_path + "/dummy.bin")

    # Create a ProtoInfo for this aspect. We have to append the target name to the
    # workspace root in order to prevent dep chains from colliding on symlinks.
    proto_info = _flatten_proto_info(
        ctx = ctx,
        proto = virtual_proto,
        descriptor_set = virtual_descriptor_set,
        deps = ctx.attr.deps,
        workspace_root = ctx.label.workspace_root,
        proto_path = proto_path,
    )

    # Create the descriptor for this specific proto.
    proto_toolchain = ctx.toolchains["@protobuf//bazel/private:proto_toolchain_type"]
    proto_common.compile(
        actions = ctx.actions,
        proto_info = proto_info,
        proto_lang_toolchain_info = proto_toolchain.proto,
        generated_files = [virtual_descriptor_set],
    )

    # Return the ProtoInfo for downstream users to use.
    return [proto_info]

proto_ros_library = rule(
    implementation = _proto_ros_library_impl,
    toolchains = [
        "@protobuf//bazel/private:proto_toolchain_type"
    ],
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

