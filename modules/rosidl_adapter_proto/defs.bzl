
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
load("@protobuf//bazel/private:cc_proto_support.bzl", "cc_proto_compile_and_link")
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

def _merge_proto_infos(ctx, name, deps, srcs = []):
    # Generating a ProtoInfo is not straightforward. There are some important limitations
    # that make it hard for an aspect to produce a ProtoInfo. You can see more info here:
    #
    #    https://github.com/protocolbuffers/protobuf/issues/23255
    #
    # So, as a compromise, what we will do is collect the dependency tree of .proto files
    # here and symlink them into a virtual directory, making them all direct sources to
    # a single ProtoInfo. This will allow a downstream consumer to use this target as a
    # dependency to `cc_proto_library` to generate bindings for another context.

    # Use the depset data structure to deduplicate the proto_infos from the whole dep tree.
    proto_files = depset(
        transitive = [
            dep[RosProtoInfo].protos for dep in deps if RosProtoInfo in dep
        ]
    ).to_list() + srcs

    # We are going to use a target-name prefixed workspace to avoid symlink collisions.
    # The name _virtual_imports/<target> is a specific structure that is supported by
    # the ProtInfo constructor when using veirtual sources!
    proto_path = "_virtual_imports/{}".format(name)

    # Create a new descriptor file for the ProtoInfo, which we'll compile on demand.
    descriptor_set = ctx.actions.declare_file("{}/{}".format(proto_path, "proto.bin"))

    # Symlink all protos into a common path to use as direct dependencies. This is a
    # requirement for the protobuf engine to function as expected.
    virtual_srcs = []
    for proto in proto_files:
        normalized_path = proto.short_path.replace("..", "external")
        src = ctx.actions.declare_file(
            normalized_path.replace(proto.owner.workspace_root, proto_path))
        ctx.actions.symlink(output = src, target_file = proto)
        virtual_srcs.append(src)

    # Construct the ProtoInfo to be returned. Note that at this point we have no deps. We
    # have effectively flattened the tree. This should be OK because cc_proto_library is
    # written to only accept one value in deps = [], so there is no chance of collision.
    proto_info = ProtoInfo(
        srcs = virtual_srcs,
        descriptor_set = descriptor_set, 
        workspace_root = ctx.label.workspace_root,
        proto_path = proto_path, 
        bin_dir = ctx.bin_dir.path,
        deps = [],
    )
    
    # Create the descriptor for the proto_info. This is a binary blob capturing all the
    # tpy information contained in the proto collection. Ideally we'd have depsets of
    # this and protos in the aspect, and merge at each node. However, the cc_proto_aspect
    # does not seem to be called when we do this.
    proto_toolchain = ctx.toolchains["@protobuf//bazel/private:proto_toolchain_type"]
    proto_common.compile(
        actions = ctx.actions,
        proto_info = proto_info,
        proto_lang_toolchain_info = proto_toolchain.proto,
        generated_files = [descriptor_set],
    )

    # Return a flattened ProtoInfo with virtual sources. We also return the proto path
    # so that if we pass this proto_info to a proto compiler we know where the output
    # artifacts will end up being written.
    return proto_info, proto_path


def _proto_aspect_impl(target, ctx):
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)

    # Generate the .proto file for the current mode.
    hdrs, srcs, proto_include_dir = generate_sources(
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

    # Generate a flattened ProtoInfo for this specific target. Note that the collected
    # .proto files will be symlinked into a virtual workspace named by this target.
    proto_info, proto_path = _merge_proto_infos(
        ctx = ctx,
        name = target.label.name,
        srcs = srcs,
        deps = ctx.rule.attr.deps,
    )

    # The files that are produced by the proto compiler will end up relative to the
    # proto_path specified in the proto_info. So, let's expect them there...
    output_proto_h = ctx.actions.declare_file(
        "{}/{}/{}/{}.pb.h".format(proto_path, package_name, message_type, message_name))
    output_proto_cc = ctx.actions.declare_file(
        "{}/{}/{}/{}.pb.cc".format(proto_path, package_name, message_type, message_name))

    # Create the C++ interface for this specific proto
    proto_toolchain = ctx.toolchains["@protobuf//bazel/private:cc_toolchain_type"].proto
    proto_common.compile(
        actions = ctx.actions,
        proto_info = proto_info,
        proto_lang_toolchain_info = proto_toolchain,
        generated_files = [output_proto_h, output_proto_cc],
        experimental_output_files = "multiple",
    )
    deps = []
    if proto_toolchain.runtime:
        deps = [proto_toolchain.runtime]
    deps.extend(getattr(ctx.rule.attr, "deps", []))
    cc_info, libraries, temps = cc_proto_compile_and_link(
        ctx = ctx,
        deps = deps,
        sources = [output_proto_cc],
        headers = [output_proto_h],
        textual_hdrs = hdrs,
        strip_include_prefix = proto_path,
    )

    # Collect all of the sources from the dependencies.
    return [
        RosProtoInfo(
            protos = depset(
                direct = srcs,
                transitive = [
                    dep[RosProtoInfo].protos
                        for dep in ctx.rule.attr.deps if RosProtoInfo in dep
                ],
            ),
        ),
        proto_info,
        cc_info
    ]


proto_aspect = aspect(
    implementation = _proto_aspect_impl,
    attr_aspects = ["deps"],
    fragments = ["proto", "cpp"],
    toolchains = use_cc_toolchain() + [
        "@protobuf//bazel/private:proto_toolchain_type",
        "@protobuf//bazel/private:cc_toolchain_type",
    ],
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
    provides = [RosProtoInfo, ProtoInfo, CcInfo],
)

def _proto_ros_library_impl(ctx):
    # Generate a flattened ProtoInfo with virtual sources.
    proto_info, _ = _merge_proto_infos(
        ctx = ctx,
        name = ctx.label.name,
        deps = ctx.attr.deps
    )

    # Generate a DefaultInfo containing all proto files.
    proto_files = []
    for dep in ctx.attr.deps:
        if RosProtoInfo in dep:
            proto_files.extend(dep[RosProtoInfo].protos.to_list())
    default_info = DefaultInfo(files = depset(proto_files))

    return [proto_info, default_info]

proto_ros_library = rule(
    implementation = _proto_ros_library_impl,
    fragments = ["proto"],
    toolchains = ["@protobuf//bazel/private:proto_toolchain_type"],
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
    provides = [ProtoInfo, DefaultInfo],
)

