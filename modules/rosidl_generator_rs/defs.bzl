
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
load("@ros//:defs.bzl", "RosInterfaceInfo")
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_aspect", "generate_sources", "generate_cc_info")
load("@rosidl_generator_c//:defs.bzl", "RosCBindingsInfo", "c_aspect")
load("@rosidl_generator_type_description//:defs.bzl", "RosTypeDescriptionInfo", "type_description_aspect")

RosRustBindingInfo = provider(
    "Encapsulates Python information generated for an underlying ROS message.", 
    fields = [
        "rs_files",
    ]
)

def _rust_aspect_impl(target, ctx):
    #print("C_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)

    # Collect all IDLs and JSON files required to generate the language bindings.
    input_idls = target[RosIdlInfo].idls.to_list()
    input_type_descriptions = target[RosTypeDescriptionInfo].jsons.to_list()

    # Generate the Python bindings
    _, rust_srcs, include_dir = generate_sources(
        ctx = ctx,
        executable = ctx.executable._rs_generator,
        mnemonic = "RustGeneration",
        input_idls = input_idls,
        input_type_descriptions = input_type_descriptions,
        input_templates = ctx.attr._rs_templates[DefaultInfo].files.to_list(),
        templates_hdrs = [], #["_{}_s.c"],
        templates_srcs = [
            "{package_name}/rust/src/msg.rs",
            "{package_name}/rust/src/lib.rs",
            "{package_name}/rust/Cargo.toml",
            "{package_name}/rust/build.rs",
        ],
        additional = ["--typesupport-impls=rosidl_typesupport_c"]
    )

    #for rust_src in rust_srcs:
    #    print(rust_src.path)

    # # Generate a cc_info for the C sources
    # cc_info_deps = [dep[CcInfo] for dep in ctx.attr._cc_deps if CcInfo in dep] 
    # cc_info_deps.extend([d for d in target[RosCBindingsInfo].cc_info.to_list()])
    # cc_info, compilation_outputs, linking_outputs = generate_cc_info(
    #     ctx = ctx,
    #     name = "{}_py".format(ctx.label.name),
    #     hdrs = [],
    #     srcs = cc_srcs,
    #     include_dirs = [include_dir],
    #     deps = cc_info_deps,
    # )

    # print(cc_info)
    # print(compilation_outputs)
    # print(linking_outputs)
    
    # Return a CcInfo provider for the aspect.
    return [
        RosRustBindingInfo(
            rs_files = depset(
                direct = rust_srcs,
                transitive = [
                    dep[RosRustBindingInfo].rs_files
                        for dep in ctx.rule.attr.deps if RosRustBindingInfo in dep
                ],
            )
        )
    ]

rust_aspect = aspect(
    implementation = _rust_aspect_impl,
    toolchains = [
        "@rules_cc//cc:toolchain_type",
        "@rules_rust//rust:toolchain_type"
    ],
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    attrs = {
        #########################################################################
        # Code generation #######################################################
        #########################################################################
        
        "_rs_generator": attr.label(
            default = Label("//:cli"),
            executable = True,
            cfg = "exec",
        ),
        "_rs_templates": attr.label(
            default = Label("//:interface_templates"),
        ),

        #########################################################################
        # Dependencies ##########################################################
        #########################################################################
        
        # "_cc_deps": attr.label_list(
        #     default = [
        #         Label("@ros//python/headers:numpy"),
        #     ],
        #     providers = [CcInfo],
        # ),  
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [
        [RosIdlInfo],
        [RosTypeDescriptionInfo],
        [RosCBindingsInfo],
    ],
    provides = [RosRustBindingInfo],
)

def _rust_ros_library_impl(ctx):
    return [
        DefaultInfo(
            files = depset(
                transitive = [
                    dep[RosRustBindingInfo].rs_files
                        for dep in ctx.attr.deps if RosRustBindingInfo in dep
                ]
            )
        )
    ]

rust_ros_library = rule(
    implementation = _rust_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_aspect,              # RosIdlInfo
                type_description_aspect, # RosTypeDescriptionInfo
                c_aspect,                # RosCBindingsInfo
                rust_aspect,             # RosRustBindingInfo
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    provides = [DefaultInfo],
)

