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
load("@rosidl_cmake//:types.bzl", "RosInterfaceInfo")
load("@rosidl_generator_c//:aspects.bzl", "c_aspect")
load("@rosidl_generator_type_description//:aspects.bzl", "type_description_aspect")
load("@rules_python//python:defs.bzl", "PyInfo")
load(":aspects.bzl", "py_aspect")
load(":types.bzl", "RosPyBindingsInfo")

def _py_ros_library_impl(ctx):
    # Collect all the message __init__.py files from dependencies and concatenate
    # them into the <package>/<type>/__init__.py files. This is so that one calls
    # "from std_msgs.msg import Time" and not "from std_msgs.msg._time import Time"
    # module_inits = {}
    # for dep in ctx.attr.deps:
    #     if RosPyBindingsInfo in dep:
    #         for init_file in dep[RosPyBindingsInfo].py_inits.to_list():
    #             init_path = "{}/__init__.py".format(dep[RosPyBindingsInfo].py_relpath)
    #             if init_path not in module_inits.keys():
    #                 module_inits[init_path] = [init_file]
    #             else:
    #                 module_inits[init_path].append(init_file)
    # init_py_files = []
    # for module_init_file, message_init_files in module_inits.items():
    #     init_py_file = ctx.actions.declare_file(module_init_file)
    #     command = "cat {} > {}".format(" ".join(
    #         [f.path for f in message_init_files]),
    #         init_py_file.path,
    #     )
    #     #print ("Command: {}".format(command))
    #     ctx.actions.run_shell(
    #         outputs = [init_py_file],
    #         inputs = message_init_files,
    #         command = command
    #     )
    #     init_py_files.append(init_py_file)

    # Symlink all generated python interface files into the runfiles tree.
    direct_sources = []

    # Handle the python interfaces
    init_file_dict = {}
    for dep in ctx.attr.deps:
        for py_interface_path, py_interface_file in dep[RosPyBindingsInfo].py_interfaces.to_list():
            symlinked_py_file = ctx.actions.declare_file(py_interface_path)
            ctx.actions.symlink( output = symlinked_py_file, target_file = py_interface_file)
            direct_sources.append(symlinked_py_file)
        for py_init_path, py_init_file in dep[RosPyBindingsInfo].py_inits.to_list():
            if py_init_path not in init_file_dict.keys():
                init_file_dict[py_init_path] = [py_init_file]
            else:
                init_file_dict[py_init_path].append(py_init_file)
    for py_init_path, py_init_files in init_file_dict.items():
        output = ctx.actions.declare_file(py_init_path)
        command = "cat {} > {}".format(" ".join(
            [f.path for f in py_init_files]),
            output.path,
        )
        ctx.actions.run_shell(
            outputs = [output],
            inputs = py_init_files,
            command = "cat {} > {}".format(" ".join([f.path for f in py_init_files]), output.path)
        )
        direct_sources.append(output)

    # Return a PyInfo provider with all generated python files and imports, and
    # a DefaultInfo provider with the dynamic libraries for C type support.
    return [
        DefaultInfo(
            runfiles = ctx.runfiles(
                transitive_files = depset(
                    transitive = [
                        dep[RosPyBindingsInfo].dynamic_libraries
                        for dep in ctx.attr.deps
                        if dep in ctx.attr.deps
                    ],
                ),
            ),
        ),
        PyInfo(
            transitive_sources = depset(
                direct = direct_sources,
                transitive = [
                    dep[PyInfo].transitive_sources
                    for dep in ctx.attr._py_deps
                    if PyInfo in dep
                ]
            ),
        ),
    ]

py_ros_library = rule(
    implementation = _py_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_aspect,  # RosIdlInfo <- RosInterfaceInfo
                type_description_aspect,  # RosTypeDescriptionInfo <- RosIdlInfo
                c_aspect,  # RosCBindingsInfo <- {RosIdlInfo, RosTypeDescriptionInfo}
                py_aspect,  # RosCcBindingsInfo <- {RosIdlInfo, RosTypeDescriptionInfo}
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
        "_py_deps": attr.label_list(
            default = [
                Label("@rosidl_parser"),
            ],
            providers = [PyInfo],
        ),
    },
    provides = [DefaultInfo, PyInfo],
)
