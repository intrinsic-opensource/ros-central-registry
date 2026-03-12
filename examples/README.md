# Examples

This folder contains a collection of examples demonstrating how to use the RCR (ROS Central Registry) Bazel rules to generate interfaces and build ROS 2 nodes in multiple languages (C, C++, and Python).

## Overview

The examples showcase a full vertical slice of ROS 2 development with Bazel:
1. **Custom Message Definition**: Defining a custom ROS 2 message (`ExampleMessage`) that locally depends on standard ROS 2 messages (`sensor_msgs/CompressedImage` and `std_msgs/String`).
2. **Interface Generation**: Generating language-specific bindings for the custom message using:
   - `c_ros_library` for C language bindings
   - `cc_ros_library` for C++ language bindings
   - `py_ros_library` for Python language bindings
   - `proto_ros_library` for generating .proto files from ROS files
3. **Node Implementation**: Implementing publisher and subscriber nodes to communicate using the generated custom message across different ROS 2 client libraries:
   - **C**: Using `rclc`
   - **C++**: Using `rclcpp`
   - **Python**: Using `rclpy`
4. **Protobuf Pipeline**: Demonstrating how to publish and subscribe using the Protobuf pipeline in C++.

## Structure

- `msg/`: Contains the custom `ExampleMessage.msg` definition and the `ros_interface` Bazel target.
- `src/`: Contains the source code for all the publisher and subscriber nodes.
- `BUILD.bazel`: Contains the Bazel targets that tie everything together, generating the artifacts and building the binaries.

## Running the Examples

You can run any of the publisher or subscriber nodes using standard Bazel commands. For instance, to start the C++ publisher:

```bash
bazel run //examples:example_ros_publisher_cc
```

And in another terminal, run a subscriber in a different language, such as Python:

```bash
bazel run //examples:example_ros_subscriber_py
```

Other available Bazel targets include:
- `//examples:example_ros_publisher_c`
- `//examples:example_ros_subscriber_c`
- `//examples:example_ros_publisher_cc`
- `//examples:example_ros_subscriber_cc`
- `//examples:example_ros_publisher_py`
- `//examples:example_ros_subscriber_py`
- `//examples:example_proto_publisher_cc`
- `//examples:example_proto_subscriber_cc`

## Selecting the Middleware Implementation

By default, RCR uses Fast RTPS (`rmw_fastrtps_cpp`). You can override this behavior and select a different RMW implementation for your nodes by passing the `--@rmw_implementation//:rmw` flag when running your Bazel targets.

The supported middleware implementations are:
- `rmw_cyclonedds_cpp`
- `rmw_fastrtps_cpp` (default)
- `rmw_fastrtps_dynamic_cpp`
- `rmw_zenoh_cpp`

For example, to run the C++ publisher with Cyclone DDS:

```bash
bazel run //examples:example_ros_publisher_cc --@rmw_implementation//:rmw=rmw_cyclonedds_cpp
```

To run it with Zenoh:

```bash
bazel run //examples:example_ros_publisher_cc --@rmw_implementation//:rmw=rmw_zenoh_cpp
```

### Note on Zenoh

When using the Zenoh middleware (`rmw_zenoh_cpp`), your nodes will require a Zenoh router to communicate correctly. You can easily start a Zenoh router using its provided Bazel target:

```bash
bazel run @rmw_zenoh_cpp//:rmw_zenohd
```

Make sure to leave the router running in the background before starting your ROS 2 publisher and subscriber nodes.
