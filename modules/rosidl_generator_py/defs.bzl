
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

load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain")
load("@rules_python//python:defs.bzl", "PyInfo")
load("@ros//:defs.bzl", "RosInterfaceInfo")
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_aspect", "generate_sources", "generate_cc_info")
load("@rosidl_generator_c//:defs.bzl", "RosCBindingsInfo", "c_aspect")
load("@rosidl_generator_type_description//:defs.bzl", "RosTypeDescriptionInfo", "type_description_aspect")

RosPyBindingInfo = provider(
    "Encapsulates Python information generated for an underlying ROS message.", 
    fields = [
        "py_info",
    ]
)

def _py_aspect_impl(target, ctx):
    #print("C_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)

    # Collect all IDLs and JSON files required to generate the language bindings.
    input_idls = target[RosIdlInfo].idls.to_list()
    input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list()

    # Generate the Python bindings
    py_srcs, cc_srcs, include_dir = generate_sources(
        ctx = ctx,
        executable = ctx.executable._py_generator,
        mnemonic = "PyGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._py_templates[DefaultInfo].files.to_list(),
        templates_hdrs = ["_{}.py"],
        templates_srcs = ["_{}_s.c"],
        additional = ["--typesupport-impls=rosidl_typesupport_c"]
    )

    # Generate a cc_info for the C sources
    cc_info_deps = [dep[CcInfo] for dep in ctx.attr._cc_deps if CcInfo in dep] 
    cc_info_deps.extend([d for d in target[RosCBindingsInfo].cc_info.to_list()])
    cc_info, compilation_outputs, linking_outputs = generate_cc_info(
        ctx = ctx,
        name = "{}_py".format(ctx.label.name),
        hdrs = [],
        srcs = cc_srcs,
        include_dirs = [include_dir],
        deps = cc_info_deps,
    )

    print(cc_info)
    print(compilation_outputs)
    print(linking_outputs)
    
    # Return a CcInfo provider for the aspect.
    return [
        DefaultInfo(
            files = depset(
                direct = py_srcs + cc_srcs,
                transitive = [
                    dep[DefaultInfo].files
                        for dep in ctx.rule.attr.deps if DefaultInfo in dep
                ],
            )
        )
        # RosPyBindingInfo(
        #     py_info = depset(
        #         direct = [py_info],
        #         transitive = [
        #             dep[RosPyBindingInfo].py_info
        #                 for dep in ctx.rule.attr.deps if RosPyBindingInfo in dep
        #         ],
        #     )
        # )
    ]

py_aspect = aspect(
    implementation = _py_aspect_impl,
    toolchains = [
        "@rules_cc//cc:toolchain_type",
        "@rules_python//python:toolchain_type"
    ],
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
                Label("@ros//python/headers:numpy"),
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
    provides = [DefaultInfo],
)

def _py_ros_library_impl(ctx):
    # # Merge the python targets
    # py_info = None
    # for dep in ctx.attr.deps:
    #     if RosPyBindingInfo in dep:
    #         for p in dep[RosPyBindingInfo].py_info.to_list():
    #             if py_info:
    #                 py_info.merge(p)
    #             else:
    #                 py_info = p

    # Merge files
    files = []
    for dep in ctx.attr.deps:
        files.extend(dep[DefaultInfo].files.to_list())

    # Return
    return [
        DefaultInfo(files = depset(files)),
    ]

py_ros_library = rule(
    implementation = _py_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_aspect,              # RosIdlInfo <- RosInterfaceInfo
                type_description_aspect, # RosTypeDescriptionInfo <- RosIdlInfo
                c_aspect,                # RosCBindingsInfo <- {RosIdlInfo, RosTypeDescriptionInfo}
                py_aspect,               # RosCcBindingsInfo <- {RosIdlInfo, RosTypeDescriptionInfo}
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [DefaultInfo],
)

