
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
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain")
load("@ros//:defs.bzl", "RosInterfaceInfo")

RosIdlInfo = provider(
    "Encapsulates IDL information generated for an underlying ROS message.", 
    fields = [
        "idl",
        "deps"
    ]
)

# Calls the tool to generate a .idl file from the interface. Inputs:
#   executable   : the script to call
#   package_name : name of the package containing the interface
#   src          : path to input interface relative to Bazel working dir
#   dst          : path to output idl relative to Bazel working dir
def _generate(ctx, executable, package_name, src, dst, mnemonic):
    ctx.actions.run(
        inputs = [src],
        outputs = [dst],
        executable = executable,
        arguments = ['-p', src.dirname, '-n', package_name, src.basename, dst.dirname],
        mnemonic = mnemonic,
        progress_message = "Generating IDL files for {}".format(ctx.label.name),
    )

def _idl_adapter_aspect_impl(target, ctx):
    #print("IDL_ROS: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)
    package_name = target.label.workspace_name.removesuffix("+")
    src = target[RosInterfaceInfo].src
    dst = ctx.actions.declare_file(
        "{package_name}/{interface_type}/{interface_name}.idl".format(
            package_name = package_name,
            interface_type = src.extension,
            interface_name = src.basename[:-len(src.extension) - 1]
        )
    )
    extra = []
    if src.extension == 'msg':
        _generate(ctx, ctx.executable._msg2idl, package_name, src, dst, "IdlFromMsg")
    elif src.extension == 'srv':
        _generate(ctx, ctx.executable._srv2idl, package_name, src, dst, "IdlFromSrv")
    elif src.extension == 'action':
        _generate(ctx, ctx.executable._action2idl, package_name, src, dst, "IdlFromAction")
    else:
        fail('Unknown file extension: ' + src.extension)
    return [
        RosIdlInfo(
            idl = dst,
            deps = depset(
                direct = [dst],
                transitive = [
                    dep[RosIdlInfo].deps for dep in ctx.rule.attr.deps if RosIdlInfo in dep
                ],
            )
        ),
    ]

# IDL aspect runs along the deps property to generate IDLs for each RosInterface,
# through one of the three cli tools, producing a ROS IDL for each one.
idl_ros_aspect = aspect(
    implementation = _idl_adapter_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_msg2idl": attr.label(
            default = Label("//:msg2idl"),
            executable = True,
            cfg = "exec",
        ),
        "_srv2idl": attr.label(
            default = Label("//:srv2idl"),
            executable = True,
            cfg = "exec",
        ),
        "_action2idl": attr.label(
            default = Label("//:action2idl"),
            executable = True,
            cfg = "exec",
        ),
    },
    required_providers = [RosInterfaceInfo],
    provides = [RosIdlInfo],
)

def _idl_ros_library_impl(ctx):
    return [
        DefaultInfo(files = depset([dep[RosIdlInfo].idl for dep in ctx.attr.deps])),
    ]

idl_ros_library = rule(
    implementation = _idl_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [idl_ros_aspect],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [DefaultInfo],
)

# Reusable tooling.


# This would be better expressed as a regex operation, but unfortunately Bazel's
# starlark language does not yet support this, and so it would require a module.
# For example: https://github.com/magnetde/starlark-re/tree/master
# https://github.com/ros2/rosidl/blob/humble/rosidl_cmake/cmake/string_camel_case_to_lower_case_underscore.cmake
def _snake_case_from_pascal_case(pascal_case):
    result = ""
    pascal_case_padded = " " + pascal_case + " "
    for i in range(len(pascal_case)):
        prev_char, char, next_char = pascal_case_padded[i:i + 3].elems()
        if char.isupper() and next_char.islower() and prev_char != " ":
            # Insert an underscore before any upper case letter which is not
            # followed by another upper case letter.
            result += "_"
        elif char.isupper() and (prev_char.islower() or prev_char.isdigit()):
            # Insert an underscore before any upper case letter which is
            # preseded by a lower case letter or number.
            result += "_"
        result += char.lower()
    return result

def _get_stem(path):
    return path.basename[:-len(path.extension) - 1]

def message_info_from_target(target):
    index = target.find("__")
    if index < 0:
        fail("Target interface {} appears malformed".format(target))
    message_type = target[:index]
    message_name = target[index+2:]
    message_code = _snake_case_from_pascal_case(message_name)
    #print("{} :: {} :: {}".format(message_type, message_name, message_code))
    return message_type, message_name, message_code

def idl_tuple_from_path(path):
    idl_parts = path.split("/")
    return "/".join(idl_parts[:-2]) + ":" + idl_parts[-2] + "/" + idl_parts[-1]

def type_description_tuple_from_path(path):
    idl_parts = path.split("/")
    idl_type = idl_parts[-2]
    idl_name = idl_parts[-1].replace(".json", ".idl")
    return "{}/{}:".format(idl_type, idl_name) + path

def generate_sources(
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
    return output_hdrs, output_srcs, _get_parent_dir(input_args.dirname)

def _get_parent_dir(path):
    return "/".join(path.split("/")[:-1])

# Merge headers, sources and deps into a CcInfo provider.
def generate_cc_info(ctx, name, hdrs, srcs, include_dirs = [], deps = []):

    # Query for the current CC toolchain and feature set.
    cc_toolchain = find_cc_toolchain(ctx)

    # Get the compiler feature configuration.
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    # Get a compilation context.
    (compilation_context, compilation_outputs) = cc_common.compile(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = srcs,
        public_hdrs = hdrs,
        compilation_contexts = [dep.compilation_context for dep in deps],
        name = name,
    )

    # Get a linking context.
    if srcs:
        linking_context, _ = cc_common.create_linking_context_from_compilation_outputs(
            actions = ctx.actions,
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
            compilation_outputs = compilation_outputs,
            linking_contexts = [dep.linking_context for dep in deps],
            name = name,
        )
    else:
        linking_context = cc_common.merge_linking_contexts(
            linking_contexts = [dep.linking_context for dep in deps]
        )

    # Assemble and return the CcInfo object.
    cc_info = CcInfo(
        compilation_context = compilation_context,
        linking_context = linking_context,
    )
    return cc_info
    