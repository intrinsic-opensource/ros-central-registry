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

def ros_local_defines(name = None):
    """Common local defines for ROS nodes"""
    return [
        "ROS_PACKAGE_NAME=\\\"{}\\\"".format(name or native.module_name()),
        "DEFAULT_RMW_IMPLEMENTATION=\"rmw_fastrtps_cpp\"",
    ]

def ros_exports_header(name, out, library = None, **kwargs):
    """Create an exports header for a ros package"""
    if not library:
        library = native.module_name()
    library = library.upper()

    content = """
#ifndef {0}_EXPORT_H
#define {0}_EXPORT_H

#ifdef {0}_STATIC_DEFINE
#  define {0}_EXPORT
#  define {0}_NO_EXPORT
#else
#  ifndef {0}_EXPORT
#    ifdef {0}_EXPORTS
        /* We are building this library */
#      define {0}_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define {0}_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef {0}_NO_EXPORT
#    define {0}_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef {0}_DEPRECATED
#  define {0}_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef {0}_DEPRECATED_EXPORT
#  define {0}_DEPRECATED_EXPORT {0}_EXPORT {0}_DEPRECATED
#endif

#ifndef {0}_DEPRECATED_NO_EXPORT
#  define {0}_DEPRECATED_NO_EXPORT {0}_NO_EXPORT {0}_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef {0}_NO_DEPRECATED
#    define {0}_NO_DEPRECATED
#  endif
#endif

#endif /* {0}_EXPORT_H */
""".format(library)

    native.genrule(
        name = name,
        outs = [out],
        cmd = "echo '{}' > $@".format(content),
        **kwargs
    )
