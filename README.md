# Overview

The goal of the [ROS Central Registry](http://intrinsic-opensource.github.io/ros-central-registry) is to provide a [Bazel](https://bazel.build) build system for [Robot Operating System (ROS)](https://ros.org) applications. Our philosophy is to build everything from source, using the dependency management system provided by the [Bazel modules](https://bazel.build/external/module) ecosystem to ensure consistency across ROS releases. In our approach, every ROS package has a corresponding Bazel module. A lot of the work in this project is inspired by Milan Vukov's [rules_ros2](https://github.com/mvukov/rules_ros2) project, one of [several existing Bazel build systems for ROS](https://github.com/RobotLocomotion/drake-ros/blob/main/bazel_ros2_rules/lib/README.md). 

> [!WARNING]
> This repository is a proof of concept and under active open development, and so no guarantees are made about stability. Please do not depend on this code until we have an official release!

# Status

The table below summarizes the status of the github workflows exercising various platforms:

| Platform          | x64                                                                                                     | arm64                                                                                                   |
| :---------------- | :-----------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------: |
| Ubuntu 24.04      | ![amd64](https://github.com/intrinsic-opensource/ros-central-registry/actions/workflows/build-test-linux-amd64.yml/badge.svg)   | ![amd64](https://github.com/intrinsic-opensource/ros-central-registry/actions/workflows/build-test-linux-arm64.yml/badge.svg)   |
| MacOS 15          | ![amd64](https://github.com/intrinsic-opensource/ros-central-registry/actions/workflows/build-test-macos-amd64.yml/badge.svg)   | ![amd64](https://github.com/intrinsic-opensource/ros-central-registry/actions/workflows/build-test-macos-arm64.yml/badge.svg)   |
| Windows 11        | ![amd64](https://github.com/intrinsic-opensource/ros-central-registry/actions/workflows/build-test-windows-amd64.yml/badge.svg) | ![amd64](https://github.com/intrinsic-opensource/ros-central-registry/actions/workflows/build-test-windows-arm64.yml/badge.svg) |

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
- Core:
  - [x] `rcl`
  - [ ] `rcl_action`
  - [ ] `rcl_lifecycle`
  - [x] `rclcpp` 
  - [ ] `rclcpp_action` 
  - [ ] `rclcpp_lifecycle` 

Our intention is to ultimately host a CI plan that responds to new ROS releases by automatically creating a set of Bazel modules and running tests. For now, however, we are working off a snapshot of the rolling release from June 2025 to determine the long term feasibility of this concept.

# Prerequisites

## Ubuntu Linux 24.04 (functional, recommended)

Right now, we only support Ubuntu 24.04. You must first install `git` and and [bazelisk](https://github.com/bazelbuild/bazelisk) in order to run or edit code. The project downloads a LLVM toolchain with clang, which means that you don't need any compiler or toolchains installed in your host environment. Th only thing you will need is a functioning build environment, because some packages use `rules_foreign_cc` and require `autoconf` to build correctly. To add this tool, run the following:

```
apt install build-essential automake autoconf libtool valgrind
```

## MacOS 15 (broken, unsupported)

You will need need to install homebrew first, and then add a few packages before trying to build any examples.

```
brew install automake autoconf libtool zstd rust bazelisk
```

The `automake`, `autoconf`, and `libtool` packages are needed by `rules_foreign_cc` to detect system features when generating Makefiles. The `zstd` package is needed by the protocol buffer rules. The `rust` package provides the `cargo` binary, which is needed for a Zenoh build. The `bazelisk` package provides the `bazel` command, which bootstraps a bazel build tool for this project.

## Windows 11 (broken, unsupported)

Currently the `toolchains_llvm` project does not offer a functional compiler setup for windows environments. For more information, please see [this issue](https://github.com/bazel-contrib/toolchains_llvm/issues/395).

# Examples

First, checkout this repository:

```
git clone https://github.com/intrinsic-opensource/ros-central-registry.git
```

If you're goign to be developing add `--recurse-submodules` but it's not necessary for just the examples.

## Custom message with minimal publisher and subscriber

We have included a small self-contained example workspace showing a minimal C++ publisher and subscriber exchanging a custom message. By default we use the FastRTPS middleware. Start by opening two terminals at the `example` directory from within this project.

In terminal 1, start a ROS node to publish a custom message.

```
cd example
bazel run //:example_ros_publisher_cc
```

In terminal 2, start a ROS node to subscribe to the custom message.

```
cd example
bazel run //:example_ros_subscriber_cc
```

We also support several other RMW  middleware implementations -- `rmw_cyclonedds_cpp`, `rmw_zenoh_cpp` and `rmw_fastrtps_dynamic_cpp`. You must specify which one you want to use, either permanently in the `.bazelrc` or as an argument to *every* Bazel action. For zenoh you must also run the `zenohd` router, like this:

```
In terminal 1, start the zenohd router.
cd example
bazel run @rmw_zenoh_cpp//:rmw_zenohd

# In terminal 2
cd example
bazel run //:example_ros_publisher_cc \
    --@rmw_implementation//:rmw=rmw_zenoh_cpp

# In terminal 3
cd example
bazel run //:example_ros_subscriber_cc \
    --@rmw_implementation//:rmw=rmw_zenoh_cpp
```

## Protocol buffer support

We have also included protocol buffer type support, which allows you to interact with messages, services and actions through the protocol buffer C++ API. The protocol buffer type adapter automatically converts to and from the protocol buffer bindings, preventing the developer from  having to write message-specific type converters, which is both laborious and prone to error.

# Setting up a development environment

ROS is a federated system, which means that its code is spread across multiple repositories. This makes managing a feature branch more challenging, because you have to keep several repositories in sync with each other. While we migrate to using the `wstool` or `vcs` tools, this repo uses submodules.

Each Bazel test is spawned in a separate sandbox environment. Since many ROS packages use the network, their tests may interfere with each other. To prevent this from happening, we force tests to use a `docker` sandbox, which runs the test in a container from the [Dockerfile](Dockerfile) image.

```
docker build -t ros-central-registry:24.04 .
```

To setup your developer environment, simply do this from the root directory of this project. This will take a bit of time recursively pulling all dependent projects to the `submodules` folders. The `submodule` folder is where our ROS packages live, while the `thirdparty` folder contains vendored dependencies.

```
git submodule update --init --recursive
```

Now, take a look at the base folder structure of this project.

```
+ .github/         # CI workflows for checking various platforms.
+ docs/            # Output modules from the scraping process.
+ example/         # Standalone example showing how to use our modules.
+ src/             # Some examples showing how to use the developer environment.
+ tools/           # Tools for transforming repos into Bazel modules.
+ submodules/      # Bazel modules we expect to always live in the RCR ***
+ thirdparty/      # Bazel modules we expect to eventually live in the BCR ***
```

Right now the developer workflow is to modify the submodules directly as you test. If you look at the `MODULE.bazel` file in the root of this project you will see that a `local_path_override` masks the Bazel module with a local copy while you test. Once you have finished making changes and testing, you must commit your changes to the `rolling-bazel` branch of each repo you modified:

As an example, lets assume you made a change to `rclcpp`. To push the change upstream, you will need write permission to the repo. Right now we are in the process of setting up a better way of handling this.

```
cd submodules/rclcpp
git pull
git checkout rolling-bazel
git commit -m "Added new tests"
git push
```

Note that you will also need to push the updated submodule commit hash on this repo, so that the developer workspace now points to a `rclcpp` repo commit containing your change. Before we do this, however, we want to first make sure that we have regenerated our bazel modules files:

```
bazel run //tools:regenerate_modules -- $PWD
```

This tool iterates over all submodules in `submodules` and `thirdparty` and calculates a diff between the upstream versioned release and the `rolling-bazel` branch, and then transforms this into Bazel module in the `docs` folder. If you run `git diff` you should see changes in `docs` to the `rclcpp` package.

Once you are happy with the change to the submodule commit and `docs` folder, you can commit and push your code to a new branch and setup a PR.

```
git checkout -b feature/rclcpp-test-fixes
git add .
git commit -m "Add some rclcpp fixes"
git push
```

When you open a pull request containing your changes, you will see some CI plans run. These let you know whether the changes you have made are functional across a variety of different platforms.

## Notes

### Module locking failures

Right now we will periodically update modules but keep the same version number. Bazel aggressively caches based on the version numbers. If you find that you are getting a download hash mismatch for modules, make sure that you remove the `MODULE.bazel.lock` file in the root workspace. If you still have issues, clean the workspace by running `bazel clean` followed by `bazel shutdown`. If you still have issues, you can try running `rm -rf $(bazel info repository_cache)` but note that this will clear all repository cache, and a build will need to re-download the LLVM toolchain.

### Module testing failures.

If you are running in docker, there is a known issue with valgrind. Before running any tests you must set your ulimit down from the default of 1073741804 to something more reasonable, like 4096. Do this with `ulimit -n 4096`, otherwise valgrind will error on initialization.

There is no straightforward way of running a full test suite across all package imports. Bazel doesn't support to support a wildcard expansion for test targets that span multiple modules. For this reason we have a [Distribution File](distribution.txt) and supporting rule in our `.bazelrc` file that enables you to run all tests across all repos in the following way:

```
bazel test --config=distribution
```