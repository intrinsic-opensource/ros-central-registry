# Overview

This repo is meant to illustrate work in progress towards a new Bazel build system for ROS in which there is a Bazel Module for each ROS package. A good chunk of the implementation is inspired by Milan Vukov's [rules_ros2](https://github.com/mvukov/rules_ros2) repository.

The ROS packages in the `modules` folder are snapshots from a rolling release, augmented with Bazel build files in a way that lends itself towards a `source.json` file format submitted to a central registry. For now, version numbers have no meaning because this project used local overrides.

Right now I have message IDL generation working up to C and C++, and I have `rcl` and `rclcpp` compiling against `rmw` with `fastrtps` only (untested). The generators also support producing `.proto` files and protocol buffer C++ interfaces using a heavily modified version of [rosidl_typesupport_protobuf](https://github.com/eclipse-ecal/rosidl_typesupport_protobuf).

# Prerequisites

The `rmw_zenoh_cpp` middleware currently depends on a Cargo build from `rules_rust`. Unfortunately, this is not hermetic and depends in turn on `rustup` being installed in your environment. To do this, simply run the following:

```
sudo apt install rustup
```

# Limitations

- [ ] Zenoh currently has compilation issues, and so it's disabled.
- [ ] ConnextDDS and GurumDDS are more complicated to support.
- [ ] FastDDS uses rules_foreign_cc and takes 400+ seconds to build.

# Examples

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

There are C and C++ example nodes with the three supported middleware implementations:

```sh
# Example C node
bazel run //:example_ros_node_c --config=rmw_cyclonedds_cpp
bazel run //:example_ros_node_c --config=rmw_fastrtps_cpp
bazel run //:example_ros_node_c --config=rmw_fastrtps_dynamic_cpp

# Example C++ node
bazel run //:example_ros_node_cc --config=rmw_cyclonedds_cpp
bazel run //:example_ros_node_cc --config=rmw_fastrtps_cpp
bazel run //:example_ros_node_cc --config=rmw_fastrtps_dynamic_cpp
```