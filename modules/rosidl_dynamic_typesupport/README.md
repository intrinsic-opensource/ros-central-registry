# rosidl_dynamic_typesupport
Unified Interface for Dynamic (Runtime) Typesupport and Serialization

This library provides a unified interface for creating and reflecting dynamic types and dynamic data based off those types **at runtime**, as well as the utilities that support those objects.

It is meant to be used with serialization support libraries that fulfill the interface.
Properly implemented, a user should be able to, given a serialized buffer, the serialization library used to serialize it, and the buffer's message/type description, obtain a way to introspect/reflect its contents.

## Example Usage

```cpp
// Init Serialization Support (in this case, using FastRTPS)
rosidl_dynamic_typesupport_serialization_support_t serialization_support;
  rosidl_dynamic_typesupport_serialization_support_init(
    rosidl_dynamic_typesupport_fastrtps_init_serialization_support_impl(),
    rosidl_dynamic_typesupport_fastrtps_init_serialization_support_interface(),
    rcutils_get_default_allocator(),
    &serialization_support);

// Configure Dynamic Type Builder
rosidl_dynamic_typesupport_dynamic_type_builder_t * flat_builder =
  rosidl_dynamic_typesupport_dynamic_type_builder_init(&serialization_support, "flat");
rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_member(flat_builder, 0, "bool_field");
rosidl_dynamic_typesupport_dynamic_type_builder_add_int32_member(flat_builder, 1, "int32_field");
rosidl_dynamic_typesupport_dynamic_type_builder_add_string_member(flat_builder, 2, "string_field");

// Create Dynamic Type
rosidl_dynamic_typesupport_dynamic_type_t * flat_type =
  rosidl_dynamic_typesupport_dynamic_type_builder_build(flat_builder);
rosidl_dynamic_typesupport_dynamic_type_builder_fini(flat_builder);

// Create Dynamic Data
rosidl_dynamic_typesupport_dynamic_data_t * flat_data =
  rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type(flat_type);

// Dynamic Data Setters
rosidl_dynamic_typesupport_dynamic_data_set_bool_value(flat_data, 0, true);
rosidl_dynamic_typesupport_dynamic_data_set_int32_value(flat_data, 1, 42);
rosidl_dynamic_typesupport_dynamic_data_set_string_value(flat_data, 2, "rar");

// Dynamic Data Getters
bool a;
int32_t b;
char * c;

rosidl_dynamic_typesupport_dynamic_data_get_bool_value(flat_data, 0, &a);  // true
rosidl_dynamic_typesupport_dynamic_data_get_int32_value(flat_data, 1, &b);  // 42
rosidl_dynamic_typesupport_dynamic_data_get_string_value(flat_data, 2, &c);  // "rar"

// Cleanup
free(c);
rosidl_dynamic_typesupport_dynamic_data_fini(flat_data);

```

## Some Terminology

- Serialization support: What this library provides; A unified, serialization library agnostic interface
- Serialization support library: The implementation of the serialization support interface, which this library calls into
- Type description (aka message description): A description of the buffer that fulfills the [rosidl](https://design.ros2.org/articles/idl_interface_definition.html) spec (this typically takes the form of the `TypeDescription` struct generated from the [type_descripion_interfaces package](https://github.com/ros2/rcl_interfaces/tree/rolling/type_description_interfaces/msg))
- Dynamic type: The serialization support library's best-effort internal representation of the type description
- Dynamic type builder: The serialization support library's means of constructing the dynamic type (on a field-by-field basis)
- Dynamic data: The serialization support library's internal representation of the serialized buffer, informed by the dynamic type

### Why **Dynamic** Typesupport?

- Dynamic: This library dynamically changes its internal behavior based off its runtime input (e.g. what serialization library is loaded to be used with it)
- Typesupport: And it supports the creation of, and access of types

This package differs from the usual ROS 2 notion of "dynamic" typesupport in that it offers **runtime** functionalities that change based off its input (hence dynamic), rather than compile-time code generation like the "dynamic" `rosidl_typesupport_introspection` package does (which generates introspectable typesupports at compile time.)

Furthermore, since this package does not actually generate any code, it is stored separately from the [rosidl repository](https://github.com/ros2/rosidl).

## The Serialization Support Interface

The serialization support interface is supposed to be an interface that is generic enough to support run-time type reflection (or in other words, dynamic type support), and is written in `C` to allow for easy language binding with any downstream libraries that use it.

Its interface is inspired by [rosidl](https://design.ros2.org/articles/idl_interface_definition.html), and the [DDS XTypes 1.3 language bindings spec (pg. 175)](https://www.omg.org/spec/DDS-XTypes/1.3/PDF), but is still intended to be generic, and so should support non-DDS middlewares.

The Serialization Support includes, for each supported middleware:

- Setting type members (dynamic type)
  - Note the lack of ability to get type members
- Getting and setting data members based off that type (dynamic data)
- Support for sequences and nested members
- Utilities

The interface makes heavy use of `void *` casting to abstract away any necessary objects used with each serialization library.

### A Note On Proper Usage

The serialization support capabilities of this library are meant to be used alongside a rosidl-compliant description of the message a buffer is meant to represent (the type description).

This is because there is a distinction between the rosidl description of a message, and what a serialization library can describe or express (in its dynamic type.)
As such, the rosidl description should be the sole source of determine what specific dynamic data getter functions are called during the unpacking of the message.
And the serialization support library is in charge of supporting mapping to and from what types it can support, and the rosidl types.

Consider the following, contrived example, where you have a message layout like so:

```
int int_field
```

Suppose you had a serialization library that only provisions support for raw bytes.
In that case, the serialization support library written would probably end up treating the `int_field` field as raw bytes when constructing its dynamic type, which means that when the message gets deserialized, something must retain the knowledge that the `int_field` was supposed to be an `int` type.

This is precisely what the type description is for, which a user should programmatically traverse, and use to determine the appropriate `int` dynamic data getter to call.
The serialization support library then should implement its `int` dynamic data getter to extract the field from the buffer (as raw bytes, since that is what the serialization library supports), and coerce the extracted type (raw bytes) to output an `int` value to return.

Additionally, this means that, in order to support such cases, the serialization library must either:

- Use the same member/field indexing as is used in the type description
- Have an alternate means to find the field described in the type description (e.g. by matching field names)

Note: There was a consideration to populate the dynamic type/data objects with the type description instead.
But as the type description and dynamic type encompasses two subtly different concerns (rosidl compliance, and serialization library specific type support, respectively), it makes sense to keep them separate.
And this is especially true given the fact that users are supposed to use the type description to call the appropriate getters.

## Type IDs

The type IDs used by this library (in `types.h`) are pulling from the [type_descripion_interfaces](https://github.com/ros2/rcl_interfaces/tree/rolling/type_description_interfaces/msg) message definitions.
