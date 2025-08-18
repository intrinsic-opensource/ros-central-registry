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

#include <rcutils/allocator.h>
#include <rcutils/types/rcutils_ret.h>
#include <rosidl_dynamic_typesupport/api/serialization_support.h>
#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>

#include "fastrtps_serialization_support.hpp"
#include "macros.hpp"

rcutils_ret_t
fastdds__serialization_support_impl_fini(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl)
{
  rcutils_allocator_t allocator = serialization_support_impl->allocator;

  auto fastdds_serialization_support_handle =
    static_cast<fastdds__serialization_support_impl_handle_t *>(
    serialization_support_impl->handle);

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    fastdds_serialization_support_handle->type_factory_->delete_instance(),
    "Could not delete dynamic type factory when finalizing serialization support");

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    fastdds_serialization_support_handle->data_factory_->delete_instance(),
    "Could not delete dynamic data factory when finalizing serialization support");

  allocator.deallocate(serialization_support_impl->handle, allocator.state);
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__serialization_support_interface_fini(
  rosidl_dynamic_typesupport_serialization_support_interface_t * serialization_support_interface)
{
  static_cast<void>(serialization_support_interface);
  return RCUTILS_RET_OK;
}
