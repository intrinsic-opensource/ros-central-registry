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
load(":types.bzl", "RosIdlInfo", "RosInterfaceInfo")

def pkg_name_and_base_from_path(path):
    """
    Extract a package name and base from a path
    """
    package_parts = path.split("/")
    package_name = package_parts[-3]
    package_base = "/".join(package_parts[:-2])
    return package_name, package_base

def snake_case_from_pascal_case(pascal_case):
    """
    # This would be better expressed as a regex operation, but unfortunately Bazel's
    # starlark language does not yet support this, and so it would require a module.
    # For example: https://github.com/magnetde/starlark-re/tree/master
    """
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

def idl_tuple_from_path(path):
    idl_parts = path.split("/")
    return "/".join(idl_parts[:-2]) + ":" + idl_parts[-2] + "/" + idl_parts[-1]

def type_description_tuple_from_path(path):
    idl_parts = path.split("/")
    idl_type = idl_parts[-2]
    idl_name = idl_parts[-1].replace(".json", ".idl")
    return "{}/{}:".format(idl_type, idl_name) + path

def generate_sources(
        target,
        ctx,
        executable,
        mnemonic,
        input_idls,
        input_templates,
        input_templates_dir,
        templates_hdrs,
        templates_srcs,
        input_type_descriptions = [],
        template_visibility_control = None,
        additional = [],
        message_is_pascal_case = True,
        debug = False):
    """
    Generic function for calling CLI tools to generate language-specific soruce files.
    """

    # Extract message metadata from the IdlInfo provider, where it was calculated.
    package_name = target[RosIdlInfo].package_name
    message_type = target[RosIdlInfo].interface_type
    message_name = target[RosIdlInfo].interface_name
    message_code = target[RosIdlInfo].interface_code

    # Some message generation retains the message case.
    if not message_is_pascal_case:
        message_code = message_name

    # The first output file is the JSON file used as args to the generator.
    input_args = ctx.actions.declare_file(
        "{}/{}_{}_{}.json".format(package_name, message_type, message_name, mnemonic),
    )

    # Prepare hdrs and srcs lists for output.
    output_hdrs = [
        ctx.actions.declare_file(
            "{}/{}/{}".format(package_name, message_type, t.format(message_code)),
        )
        for t in templates_hdrs
    ]
    output_srcs = [
        ctx.actions.declare_file(
            "{}/{}/{}".format(package_name, message_type, t.format(message_code)),
        )
        for t in templates_srcs
    ]

    # Write the generator query
    ctx.actions.write(
        input_args,
        json.encode(
            struct(
                package_name = package_name,
                idl_tuples = [idl_tuple_from_path(idl.path) for idl in input_idls],
                output_dir = input_args.dirname,
                template_dir = input_templates_dir,
                type_description_tuples = [
                    type_description_tuple_from_path(idl.path)
                    for idl in input_type_descriptions
                ],
                target_dependencies = [],
            ),
        ),
    )

    # Pass the query through the generator
    ctx.actions.run_shell(
        command = "{exec} {genfile} {extra} {out}".format(
            exec = executable.path,
            genfile = "--generator-arguments-file={}".format(input_args.path),
            extra = " ".join(additional),
            out = "" if debug else "> /dev/null 2>&1",
        ),
        tools = [executable],
        inputs = input_idls + input_type_descriptions + input_templates + [input_args],
        outputs = output_hdrs + output_srcs,
        mnemonic = mnemonic,
        progress_message = "Running {} for {}".format(mnemonic, ctx.label.name),
    )

    # Optionally include
    if template_visibility_control:
        output_c_visibility_control_stem = _get_stem(template_visibility_control)
        output_c_visibility_control_h = ctx.actions.declare_file(
            "{}/msg/{}".format(package_name, output_c_visibility_control_stem),
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

    # Collect include directories
    include_dirs = [_get_parent_dir(input_args.dirname)]

    # Return the headers and sources
    return output_hdrs, output_srcs, include_dirs

def _get_parent_dir(path):
    return "/".join(path.split("/")[:-1])

# Merge headers, sources and deps into a CcInfo provider.
def generate_compilation_information(
        ctx,
        name,
        hdrs,
        srcs,
        include_dirs = [],
        deps = [],
        header_only_deps = [],
        dynamic_dep_libraries = [],
        custom_name = None):
    """Generate language bindings

    Args:
        header_only_deps: CcInfo providers contributing headers/includes
            (via compilation_context) but NOT statically linked. Use this
            for a singleton library (e.g. rosidl_typesupport_cpp itself)
            that every generated fragment needs to *compile* against but
            must never duplicate into its own object code -- see
            dynamic_dep_libraries.
        dynamic_dep_libraries: pre-built shared library Files (e.g. the
            corresponding cc_shared_library's .dylib/.so) to link against
            dynamically instead of statically. Every message/typesupport
            fragment here gets compiled into its own standalone shared
            library; if a singleton dependency's object code were pulled in
            via deps/header_only_deps' linking_context instead, each
            fragment would get its own private copy of that dependency's
            global state (e.g. rosidl_typesupport_cpp's typesupport
            identifier constant). On Linux the ELF loader merges same-named
            globals across shared objects by default so the duplication is
            invisible; Darwin's two-level namespace does not merge them,
            so two fragments' copies of "the same" global can disagree at
            runtime. Route singleton deps through here instead of deps.
    """

    # Query for the current CC toolchain and feature set.
    cc_toolchain = find_cc_toolchain(ctx)

    # Get the compiler feature configuration.
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    # Get a compilation context
    (compilation_context, compilation_outputs) = cc_common.compile(
        name = name + "_compile",
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = srcs,
        public_hdrs = hdrs,
        compilation_contexts = [dep.compilation_context for dep in deps] +
                               [dep.compilation_context for dep in header_only_deps],
        includes = include_dirs,
    )

    static_linking_contexts = [dep.linking_context for dep in deps]

    dynamic_linking_contexts = []
    for lib_file in dynamic_dep_libraries:
        library_to_link = cc_common.create_library_to_link(
            actions = ctx.actions,
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
            dynamic_library = lib_file,
        )
        dynamic_linking_contexts.append(cc_common.create_linking_context(
            linker_inputs = depset(direct = [cc_common.create_linker_input(
                owner = ctx.label,
                libraries = depset(direct = [library_to_link]),
            )]),
        ))

    # On Darwin, singleton deps (e.g. rosidl_runtime_c) are passed via
    # header_only_deps/dynamic_dep_libraries to avoid duplicating their global
    # state across fragments under Darwin's two-level namespace. On Linux the
    # ELF loader merges same-named globals at load time, so we fall back to the
    # original approach of statically linking header_only_deps into each
    # fragment -- this keeps all runtime deps self-contained in the .so and
    # avoids needing to propagate transitive .so deps as runfiles.
    is_darwin = "apple" in cc_toolchain.target_gnu_system_name
    if is_darwin:
        all_linking_contexts = static_linking_contexts + dynamic_linking_contexts
    else:
        all_linking_contexts = static_linking_contexts + [
            dep.linking_context
            for dep in header_only_deps
        ]

    # We need a linking context so that consumers can
    linking_context, _ = cc_common.create_linking_context_from_compilation_outputs(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        compilation_outputs = compilation_outputs,
        linking_contexts = all_linking_contexts,
        name = name + "_link",
    )

    # The standard CcInfo.
    cc_info = CcInfo(
        compilation_context = compilation_context,
        linking_context = linking_context,
    )

    # We want to force linking to
    linking_outputs = cc_common.link(
        name = name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        output_type = "dynamic_library",
        compilation_outputs = compilation_outputs,
        linking_contexts = all_linking_contexts,
        user_link_flags = ["-Wl,-undefined,dynamic_lookup"] if is_darwin else [],
    )
    lib_to_link = linking_outputs.library_to_link

    # Decide if we need to symlink to the python module library.
    old_lib_file = lib_to_link.resolved_symlink_dynamic_library or lib_to_link.dynamic_library
    if custom_name:
        new_lib_file = ctx.actions.declare_file(custom_name)
        ctx.actions.symlink(
            output = new_lib_file,
            target_file = old_lib_file,
        )
        dynamic_libraries = [new_lib_file]
    else:
        dynamic_libraries = [old_lib_file]

    # Return the CcInfo and the path to the resulting dynamic library.
    return cc_info, dynamic_libraries
