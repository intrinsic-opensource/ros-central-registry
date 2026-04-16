# Copyright 2026 Open Source Robotics Foundation, Inc.
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

load("@aspect_rules_py//py:defs.bzl", "py_library")
load("@rosdistro//ament:defs.bzl", "ament_index")
load("@rosidl_cmake//:defs.bzl",
    _RosInterfaceInfo = "RosInterfaceInfo",
    _ros_interface = "ros_interface",
)
load("@rules_cc//cc:defs.bzl", "cc_library")
load(
    ":rules.bzl",
    _ros_library = "ros_library",
    _c_ros_library = "c_ros_library",
    _cc_ros_library = "cc_ros_library",
    _py_ros_library = "py_ros_library",
)

RosInterfaceInfo = _RosInterfaceInfo

ros_library = _ros_library
c_ros_library = _c_ros_library
cc_ros_library = _cc_ros_library
py_ros_library = _py_ros_library

ros_interface = _ros_interface

def core_generators(package_xml, export_name = None, deps = [], hdrs = [], includes = [], data = []):
    """Generate core interface targets

    This macro is responsible for generating all interface targets for the
    package. Some examples include:

    Args:
        package_xml: required package.xml file
        export_name: optional export name
        deps: list of dependencies for this target collection.
        hdrs: extra headers
        includes: extra includes
        data: extra data
    """

    # C bindings:
    # ----------
    # This is a rule that returns a CcInfo with compile context that sets up the consumer
    # to link against shared libraries available at runtime. It includes a DefaultInfo
    # containing all shared libraries for the whole dep chain of messages.
    c_ros_library(
        name = "c_internal",
        deps = deps,
    )
    cc_library(
        name = "c",
        hdrs = hdrs,
        includes = includes,
        deps = [":c_internal"],
    )

    # C++ bindings:
    # ------------
    # This is a rule that returns a CcInfo with compile context that sets up the consumer
    # to link against shared libraries available at runtime. It includes a DefaultInfo
    # containing all shared libraries for the whole dep chain of messages.
    cc_ros_library(
        name = "cc_internal",
        deps = deps,
    )
    cc_library(
        name = "cc",
        hdrs = hdrs,
        includes = includes,
        deps = [":cc_internal"],
    )

    # Python bindings:
    # ---------------
    # This is a rule that returns a PyInfo with python context that sets up the consumer
    # to link against shared libraries available at runtime. It includes a DefaultInfo
    # containing all shared libraries for the whole dep chain of messages.
    py_ros_library(
        name = "py_internal",
        deps = deps,
    )
    py_library(
        name = "py",
        deps = [":py_internal"],
    )

    # Message data
    # ------------
    # This is a simple rule that collects the IDLs definition files into an ament_index
    # for the calling context, so that they may be queried at runtime.
    ros_library(
        name = "ament_index_internal",
        deps = deps,
    )
    ament_index(
        name = "ament_index",
        package_xml = package_xml,
        export_name = export_name if export_name else native.module_name(),
        data = data,
        libraries = [
            ":cc",
            ":c",
            ":py"
        ]
    )