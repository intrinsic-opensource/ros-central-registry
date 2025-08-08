# Small demo for ROS C and C++ message generation using aspects

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