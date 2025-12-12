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

load("@rosidl_adapter//:aspects.bzl", "idl_aspect")
load("@rosidl_adapter//:tools.bzl", "generate_cc_info", "generate_linking_outputs", "generate_sources")
load("@rosidl_adapter//:types.bzl", "RosIdlInfo")
load("@rosidl_cmake//:types.bzl", "RosInterfaceInfo")
load("@rosidl_generator_c//:aspects.bzl", "c_aspect")
load("@rosidl_generator_c//:types.bzl", "RosCBindingsInfo")
load("@rosidl_generator_type_description//:aspects.bzl", "type_description_aspect")
load("@rosidl_generator_type_description//:types.bzl", "RosTypeDescriptionInfo")
load("@rules_python//python:defs.bzl", "PyInfo")
load(":types.bzl", "RosPyBindingsInfo")

def _py_aspect_impl(target, ctx):
    input_idls = target[RosIdlInfo].idls.to_list()
    input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list()

    # Generate the Python bindings - this proces two files (a .py and a .c source file). The
    # .c source file is a python extension which is called by the .py file.
    py_files, srcs, _ = generate_sources(
        target = target,
        ctx = ctx,
        executable = ctx.executable._py_generator,
        mnemonic = "PyGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._py_templates[DefaultInfo].files.to_list(),
        templates_hdrs = ["_{}.py", "_{}-__init__.py"],
        templates_srcs = ["_{}_s.c"],
        additional = ["--typesupport-impls=rosidl_typesupport_c"],
    )

    # Unpack the generated python files - there are two files per message. One is the
    # python interface, the other is the module initialization file (__init__.py).
    py_interface_file, py_init_file = py_files[0], py_files[1]

    # Collect the set of deps needed to build the C type support module.
    deps = [dep[CcInfo] for dep in ctx.attr._cc_deps if CcInfo in dep]
    deps.extend(target[RosCBindingsInfo].cc_infos.to_list())

    # Merge sources and deps into a CcInfo provider.
    cc_info = generate_cc_info(
        ctx = ctx,
        name = "{}_py".format(ctx.label.name),
        hdrs = [],
        srcs = srcs,
        deps = deps,
        include_dirs = [],
    )

    # Generate the linking outputs. We have to do this because Python is not aware
    # of C++ linking, and so we must manually extract the shared libraries for all
    # the dependencies and add them to the runtime folder. The shared library will
    # be called lib<pkg>__<type>__<name>.so on Linux-type platforms. For example,
    # libsensor_msgs__msg__compressed_image.so. This allows us to stitch together
    # packages containing only those messages we'd need for an application.
    linking_outputs = generate_linking_outputs(
        ctx = ctx,
        name = "{}__{}__{}".format(
            target[RosIdlInfo].package_name,
            target[RosIdlInfo].interface_type,
            target[RosIdlInfo].interface_code,
        ),
        linking_contexts = [cc_info.linking_context],
    )

    # Extract the shared library from the linking outputs. We'll need to make this
    # available in the runfiles folder, so that it can be dynamically loaded.
    dynamic_library = linking_outputs.library_to_link.dynamic_library

    # Relative paths to the generated python interface and init files. We must add
    # these, so that the rule can symlink runfiles to the correct locations.
    py_interface_path = "{}/{}/_{}.py".format(
        target[RosIdlInfo].package_name,
        target[RosIdlInfo].interface_type,
        target[RosIdlInfo].interface_code,
    )
    py_init_path = "{}/{}/__init__.py".format(
        target[RosIdlInfo].package_name,
        target[RosIdlInfo].interface_type
    )

    # Return the depset of python interfaces and extension modules. These will be
    # aggregated by the rule and placed in the runfile path as needed.
    return [
        RosPyBindingsInfo(
            py_interfaces = depset(
                direct = [(py_interface_path, py_interface_file)],
                transitive = [
                    dep[RosPyBindingsInfo].py_interfaces
                    for dep in ctx.rule.attr.deps
                    if RosPyBindingsInfo in dep
                ]
            ),
            py_inits = depset(
                direct = [(py_init_path, py_init_file)],
                transitive = [
                    dep[RosPyBindingsInfo].py_inits
                    for dep in ctx.rule.attr.deps
                    if RosPyBindingsInfo in dep
                ],
            ),
            dynamic_libraries = depset(
                direct = [dynamic_library],
                transitive = [
                    dep[RosPyBindingsInfo].dynamic_libraries
                    for dep in ctx.rule.attr.deps
                    if RosPyBindingsInfo in dep
                ],
            ),
        ),
    ]

py_aspect = aspect(
    implementation = _py_aspect_impl,
    toolchains = ["@rules_cc//cc:toolchain_type"],
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        #########################################################################
        # Code generation #######################################################
        #########################################################################
        "_py_generator": attr.label(
            default = Label("//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_py_templates": attr.label(
            default = Label("//:interface_templates"),
        ),

        #########################################################################
        # Dependencies ##########################################################
        #########################################################################
        "_cc_deps": attr.label_list(
            default = [
                Label("@rosdistro//bazel/python/cc:numpy_headers"),
                Label("@rosidl_runtime_c"),
            ],
            providers = [CcInfo],
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo],
        [RosCBindingsInfo],
    ],
    provides = [RosPyBindingsInfo],
)
