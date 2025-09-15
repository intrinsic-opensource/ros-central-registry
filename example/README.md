# Overview

This is a small example workspace showing a minimal C++ publisher and subscriber exchanging a custom message, where the message can be created and manipulated with ROS C++ bindings or protocol buffer C++ bindings. Under the hood ROS messages are always serialized to the wire, but a type adapter automatically converts to and from the protocol buffer bindings, preventing the developer from  having to write message-specify type converters, which is both laborious and prone to error.

A lot of this work was derived from [rules_ros2](https://github.com/mvukov/rules_ros2) and adapted to work with Bazel modules. If you are looking for an implementation that is more complete, we recommend that you take a look at it. Right now, our repo only includes `C` / `C++` message bindings and `rcl` / `rclcpp`. Our protobuf typesupport was adapted from [rosidl_typesupport_protobuf](https://github.com/eclipse-ecal/rosidl_typesupport_protobuf). A full list of packages is included in the [ROS Central Registry](https://asymingt.github.io/rcr-ui).

> [!WARNING]
> This repository is a proof of concept and under active open development, and so no guarantees are made about stability. Please do not depend on this code!

# System preparation

This code has been tested with `Bazel 8.3.1` via `Bazelisk` on `Ubuntu 24.04 x86_64` within a Docker container. It may work on other platforms, but our goal at this point is to create a functional first release before branching out to support other architectures and operating systems. 

The `rmw_zenoh_cpp` middleware currently depends on a Cargo build from `rules_rust`. Unfortunately, this is not hermetic and depends in turn on `rustup` being installed in your environment. In addition to a working GCC compiler, the `rules_foreign_cc` modules also requires `libtool` to function correctly. To install all three at once run the following command.

```
sudo apt install build-essential libtool rustup
```

# Examples

Clone the repo and switch to the example directory. This directory contains a Bazel workspace configured to automaticallt pull modules from the [ROS Central Registry](https://asymingt.github.io/rcr-ui) and [Bazel Central Registry](https://registry.bazel.build).

```
git clone https://github.com/asymingt/bazel_ros_demo.git
cd bazel_ros_demo/example
```

In terminal 1, start the zenohd router.

```
bazel run @rmw_zenoh_cpp//:rmw_zenohd
```

In terminal 2, start a ROS node to publish a custom proto message.

```
bazel run //:example_proto_publisher_cc
```

In terminal 3, start a ROS node to subscribe to the custom proto message.

```
bazel run //:example_proto_subscriber_cc
```

You might notice that the Bazel build system rebuilds zenohd and protobuf in the different terminals. It turns out that both modules use `rules_foreign_cc` which makes them sensitive to environment changes. So, if you open a new terminal it may trigger a rebuild. Our goal is to address this in the future.

# A note about other RMW implementations

We also support several other RMW  middleware implementations -- `rmw_cyclonedds_cpp`, `rmw_fastrtps_cpp` and `rmw_fastrtps_dynamic_cpp`. You must specify which one you want to use, either permanently in the `~/.bashrc` or as an argument to *every* bazel action. For Cyclone DDS you need not run the `zenohd` router, and it should be sufficient to do the following:

```
# In terminal 1
bazel run //:example_proto_publisher_cc \
    --@rmw_implementation//:rmw=rmw_cyclonedds_cpp

# In terminal 2
bazel run //:example_proto_subscriber_cc \
    --@rmw_implementation//:rmw=rmw_cyclonedds_cpp
```