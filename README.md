# Overview

This repo is meant to illustrate work in progress towards a new Bazel build system for ROS in which there is a Bazel Module for each ROS package. A good chunk of the implementation is inspired by Milan Vukov's [rules_ros2](https://github.com/mvukov/rules_ros2) repository.

The ROS packages in the `modules` folder are snapshots from a rolling release, augmented with Bazel build files in a way that lends itself towards a `source.json` file format submitted to a central registry. For now, version numbers have no meaning because this project used local overrides.

Right now I have message IDL generation working up to C and C++, and I have `rcl` and `rclcpp` compiling against `rmw` with `fastrtps` only (untested). The generators also support producing `.proto` files and protocol buffer C++ interfaces using a heavily modified version of [rosidl_typesupport_protobuf](https://github.com/eclipse-ecal/rosidl_typesupport_protobuf).

# Demo of C and C++ messages

Clone the repo:

```
git clone https://github.com/asymingt/bazel_ros_demo.git
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
