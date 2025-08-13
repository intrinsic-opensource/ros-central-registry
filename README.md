# Small demo for ROS C and C++ message generation using aspects

This repo is meant to illustrate work in progress towards a new Bazel build system for ROS in which there is a Bazel Module for each ROS package. A good chunk of the implementation is inspired by Milan Vukov's [rules_ros2](https://github.com/mvukov/rules_ros2) repository.

# Demo showing ROS messages being transformed to C and C++ bindings

Clone the repo:

```
git clone https://github.com/asymingt/bazel_ros_demo.git
```

Build a C target that links against ROS messages.

```
bazel build //:example_ros_c
```

Build a C++ target that links against ROS messages.

```
bazel build //:example_ros_cc
```