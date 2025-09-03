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
load(":types.bzl", "RosIdlInfo")

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

    # The last element of the traversal order of the message depset is the current element.
    src = target[RosInterfaceInfo].srcs.to_list()[-1]

    # Now, were going to transform the source file (msg, srv, action) to an IDL.
    idl = ctx.actions.declare_file(
        "{package_name}/{interface_type}/{interface_name}.idl".format(
            package_name = package_name,
            interface_type = src.extension,
            interface_name = src.basename[:-len(src.extension) - 1]
        )
    )
    if src.extension == 'msg':
        _generate(ctx, ctx.executable._msg2idl, package_name, src, idl, "IdlFromMsg")
    elif src.extension == 'srv':
        _generate(ctx, ctx.executable._srv2idl, package_name, src, idl, "IdlFromSrv")
    elif src.extension == 'action':
        _generate(ctx, ctx.executable._action2idl, package_name, src, idl, "IdlFromAction")
    else:
        fail('Unknown file extension: ' + src.extension)
    return [
        RosIdlInfo(
            idls = depset(
                direct = [idl],
                transitive = [
                    dep[RosIdlInfo].idls for dep in ctx.rule.attr.deps if RosIdlInfo in dep
                ],
            )
        ),
    ]

# IDL aspect runs along the deps property to generate IDLs for each RosInterface,
# through one of the three cli tools, producing a ROS IDL for each one.
idl_aspect = aspect(
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
