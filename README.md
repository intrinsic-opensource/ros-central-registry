# Overview

This repo illustrates work in progress towards Bazel build system for ROS, where each ROS package is a Bazel module. A good chunk of the implementation is inspired by Milan Vukov's [rules_ros2](https://github.com/mvukov/rules_ros2) repository.

The ROS packages in the `modules` folder are snapshots from a rolling release, augmented with Bazel build files in a way that lends itself towards a `source.json` file format submitted to a central registry. For now, version numbers have no meaning because this project uses local overrides. In the long term we intend to convert our patches and Bazel build files into modules in a ROS Central Registry, where the evolution of package versions and interdependency can be tightly managed.

> [!WARNING]
> This repository is a proof of concept and under active open development, and so no guarantees are made about stability. Please do not depend on this code!

# Prerequisites

The `rmw_zenoh_cpp` middleware currently depends on a Cargo build from `rules_rust`. Unfortunately, this is not hermetic and depends in turn on `rustup` being installed in your environment. To do this, simply run the following:

```
sudo apt install rustup
```

In addition to a working GCC compiler, the `rules_foreign_cc` requires `libtool` to function correctly. You must install these two packages before installing:

```
sudo apt install build-essential libtool
```

# Status

The table below summarizes the status of the github workflow exercising the target platform:

| Item              | x64                                                                                                     | arm64                                                                                                   |
| :---------------- | :-----------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------: |
| Ubuntu 24.04      | ![amd64](https://github.com/asymingt/bazel_ros_demo/actions/workflows/test-linux-amd64.yml/badge.svg)   | ![amd64](https://github.com/asymingt/bazel_ros_demo/actions/workflows/test-linux-arm64.yml/badge.svg)   |
| MacOS 15          | ![amd64](https://github.com/asymingt/bazel_ros_demo/actions/workflows/test-macos-amd64.yml/badge.svg)   | ![amd64](https://github.com/asymingt/bazel_ros_demo/actions/workflows/test-macos-arm64.yml/badge.svg)   |
| Windows 11        | ![amd64](https://github.com/asymingt/bazel_ros_demo/actions/workflows/test-windows-amd64.yml/badge.svg) | ![amd64](https://github.com/asymingt/bazel_ros_demo/actions/workflows/test-windows-arm64.yml/badge.svg) |

In terms of features, this is what we currently support:

- Message language bindings
  - [x] `c`
  - [x] `cpp`
  - [ ] `py`
  - [ ] `rust`
- Type supports:
  - [x] `rosidl_typesupport_introspection`
  - [x] `rosidl_typesupport_fastrtps`
  - [x] `rosidl_typesupport_protobuf`
- Middleware:
  - [x] `rmw_cyclonedds_cpp`
  - [x] `rmw_fastrtps_cpp`
  - [x] `rmw_fastrtps_dynamic_cpp`
  - [x] `rmw_zenoh_cpp`
  - [ ] `rmw_connextdds_cpp`
  - [ ] `rmw_gurumdds_cpp`
- Core:
  - [x] `rcl`
  - [x] `rclcpp` 

# Examples

Clone the repo, making sure to also pull all submodules:

```
git clone --recurse-submodules https://github.com/asymingt/bazel_ros_demo.git
```

Build a C target that links against ROS C bindings.

```
bazel build //:example_ros_c
```

Build a C++ target that links against ROS C++ bindings.

```
bazel build //:example_ros_cc
```

Build a C++ target that uses protocol buffer C++ bindings.

```
bazel build //:example_proto_cc
```

# Basic publisher and subscriber examples

We've included a C++ publish and subscribe examples. Note that you can set the RMW implementation with `--@rmw_implementation//:rmw=...`. Take a look at the `.bazelrc` for more information about how to do this. Since the desired RMW implementation is statically linked, switching will trigger a rebuild which can be expensive.

```sh
# In one terminal
[rolling] bazel_ros_demo 💥 bazel run //:example_ros_publisher_cc
INFO: Analyzed target //:example_ros_publisher_cc (0 packages loaded, 16770 targets configured).
INFO: Found 1 target...
Target //:example_ros_publisher_cc up-to-date:
  bazel-bin/example_ros_publisher_cc
INFO: Elapsed time: 12.685s, Critical Path: 9.65s
INFO: 131 processes: 2533 action cache hit, 4 internal, 127 processwrapper-sandbox.
INFO: Build completed successfully, 131 total actions
INFO: Running command line: bazel-bin/example_ros_publisher_cc
[INFO] [1757113510.547578338] [minimal_publisher]: Published: 'Hello, world! from C++ 0'
[INFO] [1757113511.547565265] [minimal_publisher]: Published: 'Hello, world! from C++ 1'
[INFO] [1757113512.547580653] [minimal_publisher]: Published: 'Hello, world! from C++ 2'
[INFO] [1757113513.547580236] [minimal_publisher]: Published: 'Hello, world! from C++ 3'

# In a second terminal
[rolling] bazel_ros_demo 💥 bazel run //:example_ros_subscriber_cc
INFO: Analyzed target //:example_ros_subscriber_cc (0 packages loaded, 2 targets configured).
INFO: Found 1 target...
Target //:example_ros_subscriber_cc up-to-date:
  bazel-bin/example_ros_subscriber_cc
INFO: Elapsed time: 7.668s, Critical Path: 7.24s
INFO: 6 processes: 1 action cache hit, 4 internal, 2 processwrapper-sandbox.
INFO: Build completed successfully, 6 total actions
INFO: Running command line: bazel-bin/example_ros_subscriber_cc
[INFO] [1757113577.783073521] [minimal_subscriber]: I heard: 'Hello, world! from C++ 11'
[INFO] [1757113578.783137353] [minimal_subscriber]: I heard: 'Hello, world! from C++ 12'
[INFO] [1757113579.783068847] [minimal_subscriber]: I heard: 'Hello, world! from C++ 13'
[INFO] [1757113580.783017681] [minimal_subscriber]: I heard: 'Hello, world! from C++ 14'
```

# Protobuf type support for ROS

For C++ only we have enabled protobuf type support using an updated version of the type support implementation from [eclipse-ecal/rosidl_typesupport_protobuf](https://github.com/eclipse-ecal/rosidl_typesupport_protobuf). What this means is that you can directly send protobuf types and the internal mapping to a ROS message type is handled for you. Eg:

```c++
// Include the auto-generated protobuf language bindings for C++.
#include "sensor_msgs/msg/compressed_image__typeadapter_protobuf_cpp.hpp"

// Create a publisher that takes the ::pb:: namespaced protobuf type.
auto publisher = this->create_publisher<sensor_msgs::msg::pb::CompressedImage>("topic", 10);

// Create a protobuf message and send it to the ROS pubsub syste,.
auto protobuf_message = sensor_msgs::msg::pb::CompressedImage();
publisher->publish(protobuf_message);
```

This is only possible in C++, and we've included two examples:

```sh
# In one terminal
[rolling] bazel_ros_demo 💥 bazel run //:example_ros_proto_publisher_cc
INFO: Analyzed target //:example_ros_proto_publisher_cc (0 packages loaded, 0 targets configured).
INFO: Found 1 target...
Target //:example_ros_proto_publisher_cc up-to-date:
  bazel-bin/example_ros_proto_publisher_cc
INFO: Elapsed time: 0.559s, Critical Path: 0.02s
INFO: 1 process: 4 action cache hit, 1 internal.
INFO: Build completed successfully, 1 total action
INFO: Running command line: bazel-bin/example_ros_proto_publisher_cc
[INFO] [1757115413.832288766] [minimal_proto_publisher]: Published protbuf message
[INFO] [1757115414.832286101] [minimal_proto_publisher]: Published protbuf message
[INFO] [1757115415.832314817] [minimal_proto_publisher]: Published protbuf message

# In a second terminal
[rolling] bazel_ros_demo 💥 bazel run //:example_ros_proto_subscriber_cc
INFO: Analyzed target //:example_ros_proto_subscriber_cc (0 packages loaded, 0 targets configured).
INFO: Found 1 target...
Target //:example_ros_proto_subscriber_cc up-to-date:
  bazel-bin/example_ros_proto_subscriber_cc
INFO: Elapsed time: 9.744s, Critical Path: 9.20s
INFO: 3 processes: 1 action cache hit, 1 internal, 2 processwrapper-sandbox.
INFO: Build completed successfully, 3 total actions
INFO: Running command line: bazel-bin/example_ros_proto_subscriber_cc
[INFO] [1757115413.832665806] [minimal_subscriber]: Received protobuf message
[INFO] [1757115414.832732991] [minimal_subscriber]: Received protobuf message
[INFO] [1757115415.832741286] [minimal_subscriber]: Received protobuf message
```

# Testing

There is no straightforward way of running a full test suite across all package imports. Bazel doesn't support to support a wildcard expansion for test targets that spans multiple modules. For this reason we have a [Distribution File](distribution.txt) and supporting rule in our `.bazelrc` file that enables you to run all tests across all repos. Right now this doesn't work, but ultimately we should be able to run all tests this way:

```
bazel test --config=distribution
```