// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <assert.h>
#include <stdlib.h>

#include <rosidl_dynamic_typesupport/api/serialization_support.h>
#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>
#include <rosidl_dynamic_typesupport/macros.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rcutils/types/rcutils_ret.h>

rosidl_dynamic_typesupport_serialization_support_impl_t
rosidl_dynamic_typesupport_get_zero_initialized_serialization_support_impl(void)
{
  static rosidl_dynamic_typesupport_serialization_support_impl_t zero_serialization_support_impl = {
    // .allocator  = // Initialized later
    .serialization_library_identifier = NULL,
    .handle = NULL
  };
  zero_serialization_support_impl.allocator = rcutils_get_zero_initialized_allocator();
  return zero_serialization_support_impl;
}

rosidl_dynamic_typesupport_serialization_support_interface_t
rosidl_dynamic_typesupport_get_zero_initialized_serialization_support_interface(void)
{
  static rosidl_dynamic_typesupport_serialization_support_interface_t
    zero_serialization_support_interface;  // Should be zeroed out by default

  zero_serialization_support_interface.serialization_library_identifier = NULL;
  zero_serialization_support_interface.allocator = rcutils_get_zero_initialized_allocator();
  return zero_serialization_support_interface;
}

rosidl_dynamic_typesupport_serialization_support_t
rosidl_dynamic_typesupport_get_zero_initialized_serialization_support(void)
{
  static rosidl_dynamic_typesupport_serialization_support_t zero_serialization_support = {
    // .allocator  = // Initialized later
    .serialization_library_identifier = NULL,

    // .impl  = // Initialized later
    // .methods  = // Initialized later
  };
  zero_serialization_support.allocator = rcutils_get_zero_initialized_allocator();
  zero_serialization_support.impl =
    rosidl_dynamic_typesupport_get_zero_initialized_serialization_support_impl();
  zero_serialization_support.methods =
    rosidl_dynamic_typesupport_get_zero_initialized_serialization_support_interface();

  return zero_serialization_support;
}

// CORE ============================================================================================
const char *
rosidl_dynamic_typesupport_serialization_support_get_library_identifier(
  const rosidl_dynamic_typesupport_serialization_support_t * serialization_support)
{
  return serialization_support->serialization_library_identifier;
}

rcutils_ret_t
rosidl_dynamic_typesupport_serialization_support_init(
  rosidl_dynamic_typesupport_serialization_support_impl_t * impl,
  rosidl_dynamic_typesupport_serialization_support_interface_t * methods,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(impl, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(methods, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RCUTILS_SET_ERROR_MSG("allocator is invalid");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialization_support, RCUTILS_RET_INVALID_ARGUMENT);

  if (impl->serialization_library_identifier != methods->serialization_library_identifier) {
    RCUTILS_SET_ERROR_MSG(
      "Library identifiers for serialization support impl and interface do not match");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  serialization_support->allocator = *allocator;
  serialization_support->serialization_library_identifier = impl->serialization_library_identifier;

  serialization_support->impl = *impl;
  serialization_support->methods = *methods;

  return RCUTILS_RET_OK;
}

rcutils_ret_t
rosidl_dynamic_typesupport_serialization_support_fini(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialization_support, RCUTILS_RET_INVALID_ARGUMENT);

  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
    (serialization_support->methods.serialization_support_impl_fini)(
      &serialization_support->impl)
  );

  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
    (serialization_support->methods.serialization_support_interface_fini)(
      &serialization_support->methods)
  );
  return RCUTILS_RET_OK;
}
