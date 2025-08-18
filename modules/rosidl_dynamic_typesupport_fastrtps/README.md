# rosidl_dynamic_typesupport_fastrtps
Dynamic (Runtime) Typesupport Serialization Support Library for FastRTPS

This library provides implementations for the interface specified in the `rosidl_dynamic_typesupport` package, providing FastRTPS-specific implementations for manipulating and accessing `DynamicType` and `DynamicData` objects.

```c++
rosidl_dynamic_typesupport_serialization_support_t serialization_support;

rosidl_dynamic_typesupport_serialization_support_init(
  rosidl_dynamic_typesupport_fastdds_init_serialization_support_impl(),
  rosidl_dynamic_typesupport_fastdds_init_serialization_support_interface(),
  rcutils_get_default_allocator(),
  &serialization_support);
```
