
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

def _merge_proto_infos(ctx, workspace_root, descriptor_set, srcs = [], deps = []):
    """Merge multiple ProtoInfo providers into one."""

    # Symlink the proto files relative to the workspace root
    virtual_srcs = [src for src in srcs]
    for dep in deps:
        if ProtoInfo in dep:
            for src in dep[ProtoInfo].transitive_sources.to_list():
                rel_path = "/".join(src.path.split("/")[-3:])
                if workspace_root != "":
                    rel_path = "{}/{}".format(workspace_root, rel_path)
                virtual_src = ctx.actions.declare_file(rel_path)
                ctx.actions.symlink(output = virtual_src, target_file = src)
                virtual_srcs.append(virtual_src)

    # Create the provider. Note the deps are gone,
    proto_info = ProtoInfo(
        srcs = virtual_srcs,
        deps = [],
        descriptor_set = descriptor_set,
        workspace_root = workspace_root,
    )

    # Generate the descriptor_set for the provider.
    proto_common.compile(
        actions = ctx.actions,
        proto_info = proto_info,
        proto_lang_toolchain_info = ctx.toolchains["@protobuf//bazel/private:proto_toolchain_type"].proto,
        generated_files = [descriptor_set],
    )

    # Return the merged provider.
    return proto_info
    

def _proto_aspect_impl(target, ctx):
    #print("TYPE_DESCRIPTION_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)

    # These are all the bits and pieces produced by this aspect to generate ProtoInfo and CcInfo.
    output_proto = ctx.actions.declare_file(
        "{}/{}/{}.proto".format(package_name, message_type, message_name))
    output_descriptor = ctx.actions.declare_file(
        "{}/{}/{}-descriptor-set.proto.bin".format(package_name, message_type, message_name))

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

    # Collect all proto sources into one collection. This is not efficient, but we
    # have to do it to make sure they are all relative to the same workspace_root.
    proto_info = _merge_proto_infos(
        ctx = ctx,
        workspace_root = target.label.workspace_root,
        descriptor_set = output_descriptor,
        srcs = [output_proto],
        deps = ctx.rule.attr.deps,
    )
   
    # Collect all of the sources from the dependencies.
    return [proto_info]

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
    provides = [ProtoInfo],
)

def _proto_ros_library_impl(ctx):
    proto_info = _merge_proto_infos(
        ctx = ctx,
        workspace_root = ctx.label.workspace_root,
        descriptor_set = ctx.actions.declare_file(ctx.label.name + "-descriptor-set.proto.bin"),
        deps = ctx.attr.deps,
    )
    return [proto_info]

proto_ros_library = rule(
    implementation = _proto_ros_library_impl,
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
    provides = [ProtoInfo],
)

