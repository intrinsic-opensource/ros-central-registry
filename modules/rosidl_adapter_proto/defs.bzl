
# # Copyright 2025 Open Source Robotics Foundation, Inc.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# load("@rules_cc//cc:defs.bzl", "CcInfo", "cc_common")
# load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain")
# load("@ros//:defs.bzl", "RosInterfaceInfo")

# RosIdlInfo = provider(
#     "Encapsulates IDL information generated for an underlying ROS message.", 
#     fields = [
#         "idls",
#     ]
# )

# def _idl_adapter_aspect_impl(target, ctx):
#     #print("IDL_ROS: @" + ctx.label.repo_name.removesuffix("+") + "//:" +  ctx.label.name)
#     package_name = target.label.workspace_name.removesuffix("+")

#     # The last element of the traversal order of the message depset is the current element.
#     src = target[RosInterfaceInfo].srcs.to_list()[-1]

#     # Now, were going to transform the source file (msg, srv, action) to an IDL.
#     idl = ctx.actions.declare_file(
#         "{package_name}/{interface_type}/{interface_name}.idl".format(
#             package_name = package_name,
#             interface_type = src.extension,
#             interface_name = src.basename[:-len(src.extension) - 1]
#         )
#     )
#     if src.extension == 'msg':
#         _generate(ctx, ctx.executable._msg2idl, package_name, src, idl, "IdlFromMsg")
#     elif src.extension == 'srv':
#         _generate(ctx, ctx.executable._srv2idl, package_name, src, idl, "IdlFromSrv")
#     elif src.extension == 'action':
#         _generate(ctx, ctx.executable._action2idl, package_name, src, idl, "IdlFromAction")
#     else:
#         fail('Unknown file extension: ' + src.extension)

#     # Generate the C++ bindings
#     return [
#         RosIdlInfo(
#             idls = depset(
#                 direct = [idl],
#                 transitive = [
#                     dep[RosIdlInfo].idls for dep in ctx.rule.attr.deps if RosIdlInfo in dep
#                 ],
#             )
#         ),
#     ]

# # IDL aspect runs along the deps property to generate IDLs for each RosInterface,
# # through one of the three cli tools, producing a ROS IDL for each one.
# idl_aspect = aspect(
#     implementation = _idl_adapter_aspect_impl,
#     attr_aspects = ["deps"],
#     attrs = {
#         "_cli": attr.label(
#             default = Label("//:cli"),
#             executable = True,
#             cfg = "exec",
#         ),
#         "_protoc": attr.label(
#             default = Label("@protobuf//:protoc"),
#             executable = True,
#             cfg = "exec",
#         ),
#     },
#     required_providers = [RosInterfaceInfo],
#     provides = [RosIdlInfo],
# )

# def _idl_ros_library_impl(ctx):
#     files = []
#     for dep in ctx.attr.deps:
#         files.extend(dep[RosIdlInfo].idls.to_list())
#     return [
#         DefaultInfo(files = depset(files)),
#     ]

# proto_ros_library = rule(
#     implementation = _proto_ros_library_impl,
#     attrs = {
#         "deps": attr.label_list(
#             aspects = [idl_aspect],
#             providers = [Ros],
#             allow_files = False,
#         ),
#     },
#     provides = [DefaultInfo],
# )
