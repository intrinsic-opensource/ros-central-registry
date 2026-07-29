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

"""
The general idea behind ament_index is to organize runfile data in such a way that
makes it looks like a ROS underlay. We do this by creating a symlink tree that
replicates the intended structure. For example

<runfile>
    + bin/ <-------------------------- where all the executables go
        + foo
        + bar
    + lib/ <-------------------------- where all the shared libraries go
        + libfoo.so
        + libbar.so
    + share <------------------------- where all the data goes
        + ament_index/
            + resource_index/
                + rclcpp_components/
                    + bar
                    + foo
                + packages/
                    + bar/
                        + config/
                    + foo/
                        + images/

To do this for some module "foo":

    ament_index(
        name = "data",
        components = [":component"],
        data = glob(["images/*"]),
        deps = [
            "@bar//:ament_index
        ]
    )

Importantly, the libraries in "lib" may not be dynamically linked to anything. But they
may be dlopened() by shared_library.c in rcutils. Our implementation of this logic uses
bazel runfiles to locate the "lib" subfolder when it searches for the library.

The neat thing about the ament_index is that it can be a direct layer of pkg_tar, which
makes it easy to transitively collect ament index layers into something that looks a lot
like an underlay (getting it to this point requires more work, though).
"""

# Shared library suffixes across the platforms we build for: ELF (.so, .so.1.2.3),
# Mach-O (.dylib), and PE (.dll).
def _is_shared_library(basename):
    return (
        basename.endswith(".so") or
        ".so." in basename or
        basename.endswith(".dylib") or
        basename.endswith(".dll")
    )

def _ament_index_impl(ctx):
    # When we use local path overrides for the module name, Bazel appends
    # a plus sign to the end. We must remove this, or the ament_index
    # paths will not be correct and we won't be able to resolve files.
    package_name = ctx.attr.export_name if ctx.attr.export_name else ctx.label.workspace_name.removesuffix("+")

    # Create a new, empty file for the ament index. This is a shim to help
    # the ament_index_{cpp, python, ...} discover packages. We use the
    # ctx.index.write call because it's platform independent.
    ament_index_file = ctx.actions.declare_file("_" + package_name)
    ctx.actions.write(output = ament_index_file, content = "")

    # Make sure that we have at lease
    symlinks = {}

    # FILE: package.xml
    if ctx.file.package_xml:
        symlinks["share/" + package_name + "/package.xml"] = ctx.file.package_xml

    # SHARED DATA
    #
    # `data` isn't necessarily all files belonging to *this* package:
    # rosidl_adapter_aspect's `:idl` aggregation (attr_aspects = ["deps"])
    # recurses across package boundaries, so e.g. std_msgs's own `:idl`
    # transitively includes builtin_interfaces/msg/Time.msg too (since
    # std_msgs/msg/Header.msg references it). Route each file to the share
    # tree of whichever package actually owns it -- not always package_name
    # -- using short_path, which Bazel *always* rewrites to "../<repo>/<path>"
    # for any file belonging to an external repo (bzlmod canonical repo name,
    # "+" suffix and all), regardless of whether that repo happens to be the
    # same one this ament_index target itself lives in -- so the "owned by
    # this package" check has to compare the resolved package name against
    # the ambient one, not just look for the "../" prefix.
    rosidl_interfaces = []
    if ctx.attr.data:
        for target in ctx.attr.data:
            for file in target.files.to_list():
                short_path = file.short_path
                if short_path.startswith("../"):
                    owner_repo, _, file_path = short_path[len("../"):].partition("/")
                    file_package_name = owner_repo.removesuffix("+")
                else:
                    file_package_name = package_name
                    file_path = short_path
                owned_by_this_package = file_package_name == package_name
                if owned_by_this_package and ctx.attr.data_strip_prefix:
                    prefix = ctx.attr.data_strip_prefix
                    if not prefix.endswith("/"):
                        prefix += "/"
                    file_path = file_path.removeprefix(prefix)
                symlinks["share/" + file_package_name + "/" + file_path] = file
                if owned_by_this_package and (file_path.endswith(".msg") or file_path.endswith(".srv") or file_path.endswith(".action") or file_path.endswith(".idl")):
                    rosidl_interfaces.append(file_path)
        ament_index_path = "share/ament_index/resource_index/packages/{0}".format(package_name)
        symlinks[ament_index_path] = ament_index_file
    else:
        ament_index_path = "share/ament_index/resource_index/packages/{0}".format(package_name)
        symlinks[ament_index_path] = ament_index_file

    if rosidl_interfaces:
        rosidl_file = ctx.actions.declare_file(package_name + "_rosidl_interfaces")
        ctx.actions.write(output = rosidl_file, content = "\n".join(rosidl_interfaces))
        symlinks["share/ament_index/resource_index/rosidl_interfaces/" + package_name] = rosidl_file

    # EXECUTABLES
    for target in ctx.attr.executables:
        for file in target.files.to_list():
            symlinks["bin/" + file.basename] = file

    # LIBRARIES
    for target in ctx.attr.libraries:
        for file in target.files.to_list():
            symlinks["lib/" + file.basename] = file

    # LIB_DATA
    if hasattr(ctx.attr, "lib_data"):
        for target in ctx.attr.lib_data:
            for file in target.files.to_list():
                symlinks["lib/" + package_name + "/" + file.basename] = file

    # COMPONENTS
    if ctx.attr.components:
        component_files = []
        for target in ctx.attr.components:
            all_files = depset(
                target.files.to_list(),
                transitive = [
                    target[DefaultInfo].default_runfiles.files,
                ],
            ).to_list()
            for file in all_files:
                if file.path.endswith(".component"):
                    if file not in component_files:
                        component_files.append(file)
                else:
                    symlinks["share/" + package_name + "/" + file.short_path] = file
                    if _is_shared_library(file.basename):
                        symlinks["lib/" + file.basename] = file
        if component_files:
            out_comp_file = ctx.actions.declare_file("share/ament_index/resource_index/rclcpp_components/" + package_name)
            ctx.actions.run_shell(
                inputs = component_files,
                outputs = [out_comp_file],
                command = "cat " + " ".join([f.path for f in component_files]) + " > " + out_comp_file.path,
            )
            symlinks["share/ament_index/resource_index/rclcpp_components/" + package_name] = out_comp_file

    # PLUGINS
    if ctx.files.plugins:
        desc_paths = []
        pkg_path = package_name + "/"
        for f in ctx.files.plugins:
            rel_path = f.short_path
            if rel_path.startswith(pkg_path):
                rel_path = rel_path[len(pkg_path):]
            else:
                rel_path = f.basename
            target_path = "share/" + package_name + "/" + rel_path
            symlinks[target_path] = f
            desc_paths.append(target_path)
        resource_file = ctx.actions.declare_file(package_name + "_resource")
        ctx.actions.write(output = resource_file, content = "\n".join(desc_paths))

        # plugin_category is the interface package name that owns the class loader
        # (e.g. "rosbag2_storage" for plugins that implement rosbag2_storage interfaces).
        # Pluginlib discovers plugins by searching resource_index/<category>__pluginlib__plugin/.
        # If not specified, fall back to the current package name (self-registration).
        plugin_category = ctx.attr.plugin_category if ctx.attr.plugin_category else package_name
        symlinks["share/ament_index/resource_index/" + plugin_category + "__pluginlib__plugin/" + package_name] = resource_file

    # ament_index_cpp's runfiles-fallback search path (get_search_paths.cpp)
    # only ever looks under <runfiles>/_main (bzlmod's fixed canonical name
    # for the root module -- a constant, not this project's own name),
    # regardless of which repo's target declared the data or which repo the
    # running binary/test belongs to -- so "share/" package data
    # (package.xml, resource_index entries, .msg/.srv/.idl text,
    # component/plugin registrations) must land at exactly
    # <runfiles>/_main/share/..., unnamespaced by the declaring repo
    # (root_symlinks, with an explicit "_main/" prefix -- root_symlinks on
    # its own places entries at <runfiles>/<path> with no prefix at all),
    # mimicking a single real ROS underlay's flat share tree. "bin/"+"lib/"
    # stay namespaced under the owning repo (regular symlinks):
    # cc_shared_library's own dynamic-linking machinery (_solib_*, RPATH)
    # resolves sibling .so's relative to the consuming target's own repo
    # subtree, unrelated to ament_index_cpp's lookup -- flattening those too
    # breaks dlopen-based RMW implementation loading.
    share_symlinks = {}
    other_symlinks = {}
    for path, file in symlinks.items():
        if path.startswith("share/"):
            share_symlinks["_main/" + path] = file
        else:
            other_symlinks[path] = file

    # Return a collection of runfiles with manipulated symlinks.
    transitive_runfiles = [dep[DefaultInfo].default_runfiles for dep in ctx.attr.deps]
    return [
        DefaultInfo(
            files = depset(
                direct = symlinks.values(),
                transitive = [dep[DefaultInfo].files for dep in ctx.attr.deps],
            ),
            runfiles = ctx.runfiles(
                symlinks = other_symlinks,
                root_symlinks = share_symlinks,
            ).merge_all(transitive_runfiles),
        ),
    ]

ament_index_rule = rule(
    implementation = _ament_index_impl,
    attrs = {
        "deps": attr.label_list(providers = [DefaultInfo]),
        "export_name": attr.string(),
        "package_xml": attr.label(allow_single_file = True),
        "executables": attr.label_list(allow_files = True),
        "headers": attr.label_list(allow_files = True),
        "libraries": attr.label_list(allow_files = True),
        "data": attr.label_list(allow_files = True),
        "data_strip_prefix": attr.string(),
        "lib_data": attr.label_list(allow_files = True),
        "components": attr.label_list(allow_files = True),
        "plugins": attr.label_list(allow_files = True),
        "plugin_category": attr.string(),
    },
)

def ament_index(name = None, **kwargs):
    ament_index_rule(name = name if name else "ament_index", **kwargs)
