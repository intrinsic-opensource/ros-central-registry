
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
load("@rosidl_adapter//:defs.bzl", "RosIdlInfo", "idl_ros_aspect")

TYPE_DESCRIPTION_GENERATOR_TEMPLATES = ["{}.json"]

RosTypeDescriptionInfo = provider(
    "Encapsulates type description information generated for an underlying IDL.", 
    fields = [
        "files",
    ]
)

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
    idl_tuple = "/".join(idl_parts[:-2]) + ":" + idl_parts[-2] + "/" + idl_parts[-1]
    return idl_tuple

def pkg_name_and_base_from_path(path):
    package_parts = path.split("/")
    package_name = package_parts[-3]
    package_base = "/".join(package_parts[:-2])
    return package_name, package_base

def _type_decription_idl_aspect_impl(target, ctx):
    #print("TYPE_DESCRIPTION_IDL: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)
    package_name = ctx.label.repo_name.removesuffix("+")
    message_type, message_name, message_code = message_info_from_target(ctx.label.name)

    # Collects a sequence of tuples containing the package base path and interface name.
    idl_tuples = []
    input_idls = []

    # We must first collect the dependency JSONs
    include_paths = {}
    for dep in ctx.rule.attr.deps:
        for idl in dep[RosIdlInfo].idls:
            package_name, package_base = pkg_name_and_base_from_path(idl.path)
            include_paths[package_name] = package_base
            idl_tuples.append(idl_tuple_from_path(idl.path))
            input_idls.append(idl)

    # if this is a service, we need additional IDLs...
    if message_type == "srv":
        service_label = ctx.attr._service_idls
        for idl in service_label[DefaultInfo].files.to_list():
            package_name, package_base = pkg_name_and_base_from_path(idl.path)
            include_paths[package_name] = package_base
            idl_tuples.append(idl_tuple_from_path(idl.path))
            input_idls.append(idl)

    # If this is an action we need additional IDLs...
    if message_type == "action":
        action_label = ctx.attr._action_idls
        for idl in action_label[DefaultInfo].files.to_list():
            package_name, package_base = pkg_name_and_base_from_path(idl.path)
            include_paths[package_name] = package_base
            idl_tuples.append(idl_tuple_from_path(idl.path))
            input_idls.append(idl)

    # Now add this package's IDLs
    for idl in target[RosIdlInfo].idls:
        package_name, package_base = pkg_name_and_base_from_path(idl.path)
        include_paths[package_name] = package_base
        idl_tuples.append(idl_tuple_from_path(idl.path))
        input_idls.append(idl)
        
    # print("#################### {}//{}".format(package_name, message_name))
    # print("include_paths: {}".format(include_paths))
    # print("idl_tuples: {}".format(idl_tuples))
    # print("input_idls: {}".format(input_idls))
    # print("####################")

    # Collect inputs
    input_templates = ctx.attr._rosidl_templates[DefaultInfo].files

    # The first output file is the JSON file used as args to the generator.
    json_file = ctx.actions.declare_file(
        "{}/{}_{}_type_description.json".format(package_name, message_type, message_name))
    json_data = json.encode(
        struct(
            package_name = package_name,
            idl_tuples = idl_tuples,
            output_dir = json_file.dirname,
            template_dir = input_templates.to_list()[0].dirname,
            include_paths = ["{}:{}".format(k,v) for k, v in include_paths.items()],
        )
    )
    ctx.actions.write(json_file, json_data)
    
    # Iterate over all the IDLs included in the bundle required for this
    # target, and make sure we have declared output files for each one.
    output_files = []
    for idl in target[RosIdlInfo].idls:
        for template in TYPE_DESCRIPTION_GENERATOR_TEMPLATES:
            generated_file = "{}/{}/{}".format(
                package_name,
                message_type,
                template.format(message_name),
            )
            output_files.append(ctx.actions.declare_file(generated_file))

    # Run the action to generate the files
    ctx.actions.run(
        inputs = target[RosIdlInfo].idls + input_idls + input_templates.to_list() + [json_file],
        outputs = output_files,
        executable = ctx.executable._rosidl_generator,
        arguments = [
            "--generator-arguments-file={}".format(json_file.path),
        ],
        mnemonic = "IdlToJSON",
        progress_message = "Generating Type Description for {}".format(ctx.label.name),
    )

    # Collect all of the sources from the dependencies.
    files = output_files
    for dep in ctx.rule.attr.deps:
        files.extend(dep[RosTypeDescriptionInfo].files.to_list())
    return [
        RosTypeDescriptionInfo(files = depset(files))
    ]


type_description_idl_aspect = aspect(
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
        "_service_idls": attr.label(
            default = Label("@rosidl_adapter//:service_idls"),
        ),
        "_action_idls": attr.label(
            default = Label("@rosidl_adapter//:action_idls"),
        ),
    },
    required_providers = [RosInterfaceInfo],
    required_aspect_providers = [RosIdlInfo],
    provides = [RosTypeDescriptionInfo],
)

def _type_description_ros_library_impl(ctx):
    files = []
    for dep in ctx.attr.deps:
        files.extend(dep[RosTypeDescriptionInfo].files.to_list())
    return [
        DefaultInfo(files = depset(files)),
    ]

type_description_ros_library = rule(
    implementation = _type_description_ros_library_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [
                idl_ros_aspect,                 # idl  <- ros aspect [STEP 1]
                type_description_idl_aspect,    # json <- idl aspect [STEP 2]
            ],
            providers = [RosInterfaceInfo],
            allow_files = False,
        ),
    },
    #provides = [CcInfo],
)

