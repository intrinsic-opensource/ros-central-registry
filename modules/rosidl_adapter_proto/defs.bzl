
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

load("@rules_cc//cc:find_cc_toolchain.bzl", "use_cc_toolchain")
load("@ros//:defs.bzl", "RosInterfaceInfo")
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_aspect",
    "message_info_from_target", "idl_tuple_from_path", "generate_sources", "generate_cc_info")

RosProtoInfo = provider(
    "Encapsulates protobuf information generated for an underlying IDL.", 
    fields = [
        "protos",
        "cc_files",
        "cc_infos",
    ]
)

def pkg_name_and_base_from_path(path):
    package_parts = path.split("/")
    package_name = package_parts[-3]
    package_base = "/".join(package_parts[:-2])
    return package_name, package_base

def _proto_aspect_impl(target, ctx):
    #print("TYPE_DESCRIPTION_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)

    # Generate the C++ bindings
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

    # Aggregate all proto paths together, so that a dependent proto can find its parents.
    proto_files = []
    for dep in ctx.rule.attr.deps:
        if RosProtoInfo in dep:
            for proto in dep[RosProtoInfo].protos.to_list():
                proto_files.append(proto)

    # Call protoc to generate the C++ files from the proto file. We'd normally do this
    # in a separate rule, but we need the typesupport aspects to be able to generate it.
    output_proto = proto_srcs[0]
    output_proto_h = ctx.actions.declare_file(
        "{}/{}/{}.pb.h".format(package_name, message_type, message_name))
    output_proto_cc = ctx.actions.declare_file(
        "{}/{}/{}.pb.cc".format(package_name, message_type, message_name))
    arguments = [
        '--proto_path={}'.format("/".join(p.dirname.split("/")[:-2])) for p in proto_files
    ] + [
        "--proto_path=.",
        "--cpp_out=dllexport_decl=ROSIDL_ADAPTER_PROTO_PUBLIC__{}:.".format(package_name),
        output_proto.path
    ]
    ctx.actions.run(
        executable = ctx.executable._protoc,
        arguments = arguments,
        inputs = proto_files + [output_proto],
        outputs = [output_proto_h, output_proto_cc],
        mnemonic = "ProtoCompile",
        progress_message = "Calling protoc to compile {}".format(ctx.label.name),
    )

    # These deps will all have CcInfo providers.
    deps = []
    for dep in ctx.rule.attr.deps:
        if RosProtoInfo in dep:
            deps.extend([d for d in dep[RosProtoInfo].cc_infos.to_list()])

    # Merge headers, sources and deps into a CcInfo provider.
    cc_info = generate_cc_info(
        ctx = ctx,
        name = "{}_proto".format(ctx.label.name),
        hdrs = proto_hdrs + [output_proto_h],
        srcs = [output_proto_cc],
        include_dirs = [proto_include_dir],
        deps = deps,
    )

    # Collect all of the sources from the dependencies.
    return [
        RosProtoInfo(
            protos = depset(
                direct = [output_proto],
                transitive = [
                    dep[RosProtoInfo].protos
                        for dep in ctx.rule.attr.deps if RosProtoInfo in dep
                ],
            ),
            cc_files = depset(
                direct = [output_proto_h, output_proto_cc],
                transitive = [
                    dep[RosProtoInfo].cc_files
                        for dep in ctx.rule.attr.deps if RosProtoInfo in dep
                ],
            ),
            cc_infos = depset(
                direct = [cc_info],
                transitive = [
                    dep[RosProtoInfo].cc_infos
                        for dep in ctx.rule.attr.deps if RosProtoInfo in dep
                ],
            ),
        ),
    ]


proto_aspect = aspect(
    implementation = _proto_aspect_impl,
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    toolchains = use_cc_toolchain(),
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
        "_protoc": attr.label(
            default = Label("@protobuf//:protoc"),
            executable = True,
            cfg = "exec"
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [RosIdlInfo],
    provides = [RosProtoInfo],
)

def _proto_ros_library_impl(ctx):
    files = []
    for dep in ctx.attr.deps:
        files.extend(dep[RosProtoInfo].protos.to_list())
    return [
        DefaultInfo(files = depset(files)),
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
    provides = [DefaultInfo],
)

