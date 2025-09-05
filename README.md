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

We've included a C++ publish and susbscribe examples. Note that you can replace `--config=rmw_cyclonedds_cpp` with `--config=rmw_fastrtps_cpp` or `--config=rmw_fastrtps_dynamic_cpp` to switch between middleware implementations as needed. We intend to add Zenoh support in the near future.

```sh
# In one terminal
[shell] bazel run //:example_ros_publisher_cc --config=rmw_cyclonedds_cpp
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
[shell] bazel run //:example_ros_subscriber_cc --config=rmw_cyclonedds_cpp
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
