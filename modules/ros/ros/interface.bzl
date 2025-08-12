
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

RosInterfaceInfo = provider(
    "Provides info for interface code generation.",
    fields = {
        "src": "The source file defining an interface.",
        "deps": "The interfaces on which this interface depends.",
    }
)

def _ros_interface_impl(ctx):
    return RosInterfaceInfo(
        src = ctx.file.src,
        deps = depset(
            direct = [
                dep[RosInterfaceInfo].src for dep in ctx.attr.deps
            ],
            transitive = [
                dep[RosInterfaceInfo].deps for dep in ctx.attr.deps
            ],
        )
    )

ros_interface = rule(
    implementation = _ros_interface_impl,
    attrs = {
        "src": attr.label(
            allow_single_file = [
                ".msg",
                ".srv",
                ".action"
            ],
            mandatory = True,
        ),
        "deps": attr.label_list(providers = [RosInterfaceInfo]),
    },
    provides = [RosInterfaceInfo],
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
    return "/".join(idl_parts[:-2]) + ":" + idl_parts[-2] + "/" + idl_parts[-1]

def type_description_tuple_from_path(path):
    idl_parts = path.split("/")
    idl_type = idl_parts[-2]
    idl_name = idl_parts[-1].replace(".json", ".idl")
    return "{}/{}:".format(idl_type, idl_name) + path
