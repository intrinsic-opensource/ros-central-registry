
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

load("@ros//:defs.bzl", "RosInterfaceInfo")
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_aspect",
    "message_info_from_target", "idl_tuple_from_path")

TYPE_DESCRIPTION_GENERATOR_TEMPLATES = ["{}.json"]

RosTypeDescriptionInfo = provider(
    "Encapsulates type description information generated for an underlying IDL.", 
    fields = [
        "jsons",
    ]
)

def pkg_name_and_base_from_path(path):
    package_parts = path.split("/")
    package_name = package_parts[-3]
    package_base = "/".join(package_parts[:-2])
    return package_name, package_base

def _type_decription_idl_aspect_impl(target, ctx):
    #print("TYPE_DESCRIPTION_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)

    # This is the single file we'll be generating as part of this aspect call.
    output_json = ctx.actions.declare_file(
        "{}/{}/{}".format(
            package_name,
            message_type,
            "{}.json".format(message_name),
        )
    )

    # However, generating the single file above requires that we generate IDLs
    # and JSONs for all message that this one depends on. THe way to do this
    # is to recursively call up the tree storing depsets as we go...
    input_idls = target[RosIdlInfo].idls.to_list()
    input_templates = ctx.attr._rosidl_templates[DefaultInfo].files.to_list()

    # Now add this package's IDL
    idl_tuples = []
    include_paths = {}
    for idl in input_idls:
        msg_package_name, msg_package_base = pkg_name_and_base_from_path(idl.path)
        include_paths[msg_package_name] = msg_package_base
        idl_tuples.append(idl_tuple_from_path(idl.path))

    # The first output file is the JSON file used as args to the generator.
    input_args = ctx.actions.declare_file(
        "{}/{}_{}_type_description.json".format(package_name, message_type, message_name))
    ctx.actions.write(
        input_args,
        json.encode(
            struct(
                package_name = package_name,
                idl_tuples = idl_tuples,
                output_dir = input_args.dirname,
                template_dir = input_templates[0].dirname,
                include_paths = ["{}:{}".format(k,v) for k, v in include_paths.items()],
            )
        )
    )

    # Run the action to generate the files
    ctx.actions.run(
        inputs = input_idls + input_templates + [input_args],
        outputs = [output_json],
        executable = ctx.executable._rosidl_generator,
        arguments = ["--generator-arguments-file={}".format(input_args.path)],
        mnemonic = "IdlToTypeDescription",
        progress_message = "Generating Type Description for {}".format(ctx.label.name),
    )

    # Collect all of the sources from the dependencies.
    return [
        RosTypeDescriptionInfo(
            jsons = depset(
                direct = [output_json],
                transitive = [
                    dep[RosTypeDescriptionInfo].jsons
                        for dep in ctx.rule.attr.deps
                            if RosTypeDescriptionInfo in dep],
            ),
        )
    ]


type_description_aspect = aspect(
    implementation = _type_decription_idl_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_rosidl_generator": attr.label(
            default = Label("//:rosidl_generator"),
            executable = True,
            cfg = "exec",
        ),
        "_rosidl_templates": attr.label(
            default = Label("//:templates"),
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [RosIdlInfo],
    provides = [RosTypeDescriptionInfo],
)

def _type_description_ros_library_impl(ctx):
    files = []
    for dep in ctx.attr.deps:
        files.extend(dep[RosTypeDescriptionInfo].jsons.to_list())
    return [
        DefaultInfo(files = depset(files)),
    ]

type_description_ros_library = rule(
    implementation = _type_description_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_aspect,                 # idl  <- ros aspect [STEP 1]
                type_description_aspect,    # json <- idl aspect [STEP 2]
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    #provides = [CcInfo],
)

