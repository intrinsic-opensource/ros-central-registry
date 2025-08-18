// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS__VISIBILITY_CONTROL_H_
#define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus


// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_EXPORT __attribute__ ((dllexport))
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_EXPORT __declspec(dllexport)
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_IMPORT __declspec(dllimport)
  #endif // ifdef __GNUC__
  #ifdef ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_BUILDING_DLL
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_EXPORT
  #else
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_IMPORT
  #endif // ifdef ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_BUILDING_DLL
  #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_LOCAL
#else
  #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_EXPORT __attribute__ ((visibility("default")))
  #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_IMPORT
  #if __GNUC__ >= 4
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC __attribute__ ((visibility("default")))
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
    #define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_LOCAL
  #endif // if __GNUC__ >= 4
#endif // if defined _WIN32 || defined __CYGWIN__


#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif  // ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS__VISIBILITY_CONTROL_H_
