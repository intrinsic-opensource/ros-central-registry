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

    # Each generated fragment is compiled into its own shared library, so it is
    # always the "owner" of the symbols its rosidl visibility_control header
    # guards. On Windows that header resolves <PREFIX>_PUBLIC[_<pkg>] to
    # dllimport unless the corresponding <PREFIX>_BUILDING_DLL[_<pkg>] macro is
    # defined while compiling the fragment's own sources -- and clang rejects a
    # dllimport attribute on a (non-inline) function *definition*, so every
    # message package fails to compile without it. Derive the prefix from
    # `name`, which each rosidl aspect formats as
    # "<pkg>__<interface_type>__<interface_code>__<generator>" (e.g.
    # "builtin_interfaces__msg__Duration__rosidl_generator_c"): the generator
    # segment uppercased is exactly the visibility macro prefix
    # (ROSIDL_GENERATOR_C, ROSIDL_TYPESUPPORT_CPP, ...). The generators disagree
    # on whether BUILDING_DLL is namespaced by package: rosidl_generator_c/cpp
    # and the fastrtps typesupports use <PREFIX>_BUILDING_DLL_<pkg>, while the
    # plain and introspection typesupports use a bare <PREFIX>_BUILDING_DLL. We
    # define both forms; the unused one is harmless. No-op on non-Windows
    # platforms, where the header's dllexport/dllimport block is guarded by
    # `defined _WIN32 || defined __CYGWIN__`.
    local_defines = []
    name_parts = name.split("__")
    if len(name_parts) >= 4:
        prefix = name_parts[-1].upper()
        local_defines = [
            "{}_BUILDING_DLL".format(prefix),
            "{}_BUILDING_DLL_{}".format(prefix, name_parts[0]),
            # The protobuf generators (rosidl_adapter_proto,
            # rosidl_typesupport_protobuf_c/cpp) namespace the macro with a
            # double underscore: <PREFIX>_BUILDING_DLL__<pkg>.
            "{}_BUILDING_DLL__{}".format(prefix, name_parts[0]),
        ]

    # Get a compilation context
    (compilation_context, compilation_outputs) = cc_common.compile(
        name = name + "_compile",
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = srcs,
        public_hdrs = hdrs,
        local_defines = local_defines,
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
    # Windows needs the same dynamic treatment as Darwin, for a different
    # reason: a singleton's public symbols (e.g. the typesupport_identifier
    # constant) are declared __declspec(dllimport) in its headers, so a fragment
    # that references them must import them from the singleton's DLL. Statically
    # linking the singleton's archive instead leaves the dllimport reference
    # unsatisfied -- ld.lld reports "undefined symbol: __declspec(dllimport) ..."
    # and notes the symbol is in the .a "but cannot be used because it is not an
    # import library". Routing the singleton through dynamic_dep_libraries makes
    # the fragment link against the .dll and import the symbol.
    is_darwin = "apple" in cc_toolchain.target_gnu_system_name
    is_windows = "windows" in cc_toolchain.target_gnu_system_name or "mingw" in cc_toolchain.target_gnu_system_name
    if is_darwin or is_windows:
        all_linking_contexts = static_linking_contexts + dynamic_linking_contexts
    else:
        all_linking_contexts = static_linking_contexts + [
            dep.linking_context
            for dep in header_only_deps
        ]

    # Link the fragment into its own shared library. Done before building the
    # consumer-facing linking context below so that on Windows that context can
    # point at this DLL.
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

    # We need a linking context so that consumers can link against this fragment.
    # On Windows each fragment's public symbols are __declspec(dllimport) to its
    # consumers (a dependent typesupport fragment, the next package's messages,
    # ...), so a consumer that statically linked this fragment's object code
    # would leave those imports unsatisfied -- ld.lld reports "undefined symbol:
    # __declspec(dllimport) <pkg>__msg__<Msg>__init" and similar. Hand consumers
    # a context that imports from this fragment's DLL instead; the fragment's own
    # transitive contexts (its deps' DLLs and the dynamic singletons) are merged
    # in so they propagate too. Elsewhere keep bundling the objects into the
    # consumer (Linux merges duplicate globals at load time; Darwin routes the
    # singletons through dynamic_dep_libraries).
    if is_windows:
        own_library = cc_common.create_library_to_link(
            actions = ctx.actions,
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
            dynamic_library = old_lib_file,
        )
        own_context = cc_common.create_linking_context(
            linker_inputs = depset(direct = [cc_common.create_linker_input(
                owner = ctx.label,
                libraries = depset(direct = [own_library]),
            )]),
        )
        linking_context = cc_common.merge_linking_contexts(
            linking_contexts = [own_context] + all_linking_contexts,
        )
    else:
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

    # Return the CcInfo and the path to the resulting dynamic library.
    return cc_info, dynamic_libraries
