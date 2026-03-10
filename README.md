<p align="center">
<img width="256" src="docs/source/rcr_logo.png"/>
<br/>
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/ubuntu-24.04_bazel-8.5.1_x86_64.json">
&nbsp;
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/ubuntu-24.04_bazel-9.0.0_x86_64.json">
<br/>
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/ubuntu-24.04_bazel-8.5.1_aarch64.json">
&nbsp;
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/ubuntu-24.04_bazel-9.0.0_aarch64.json">
<br/>
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/macos-26_bazel-8.5.1_x86_64.json">
&nbsp;
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/macos-26_bazel-9.0.0_x86_64.json">
<br/>   
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/macos-26_bazel-8.5.1_aarch64.json">
&nbsp;
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/macos-26_bazel-9.0.0_aarch64.json">
</p>

# Overview
The [ROS Central Registry](http://intrinsic-opensource.github.io/ros-central-registry) provides [Bazel](https://bazel.build) modules for [Robot Operating System (ROS)](https://ros.org) core packages. Our philosophy is to build everything from source, using the dependency management system provided by the [Bazel](https://bazel.build) ecosystem to ensure consistency across ROS releases. A lot of the work in this project is inspired by Milan Vukov's [rules_ros2](https://github.com/mvukov/rules_ros2) project, one of [several existing Bazel build systems for ROS](https://github.com/RobotLocomotion/drake-ros/blob/main/bazel_ros2_rules/lib/README.md). 

> [!WARNING]
> This repository is a proof of concept and under active open development, and so no guarantees are made about stability. Please do not depend on this code until we have an official release!

# Status

In terms of features, this is what we currently support:

- Message generators:
  - [x] `c`
  - [x] `cpp`
  - [x] `py`
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
- Client libraries:
  - [x] `rcl`
  - [x] `rcl_action`
  - [x] `rcl_lifecycle`
  - [x] `rclc`
  - [x] `rclc_lifecycle`
  - [x] `rclc_parameter`
  - [x] `rclcpp` 
  - [x] `rclcpp_action` 
  - [x] `rclcpp_lifecycle`
  - [x] `rclpy`
  - [ ] `rclrs`

Our intention is to ultimately host a CI plan that responds to new ROS releases by automatically creating a set of Bazel modules and running tests. For now, however, we are working off a snapshot of the rolling release from June 2025 to determine the long term feasibility of this concept.

# Prerequisites

Right now, we only support Ubuntu 24.04. You must first install `git` and and [bazelisk](https://github.com/bazelbuild/bazelisk) in order to run or edit code. The project downloads a LLVM toolchain with clang, which means that you don't need any compiler or toolchains installed in your host environment.

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

### Airlock / pip failures

If you are in a corporate environment, you might need to configure pip correctly before you can use this repo. I would consider looking at the documentation for `airlock` and run `gcert` and `gpkg setup` before trying to build or run code in this repo.

### Module locking failures

Right now we will periodically update modules but keep the same version number. Bazel aggressively caches based on the version numbers. If you find that you are getting a download hash mismatch for modules, make sure that you remove the `MODULE.bazel.lock` file in the root workspace. If you still have issues, clean the workspace by running `bazel clean` followed by `bazel shutdown`. If you still have issues, you can try running `rm -rf $(bazel info repository_cache)` but note that this will clear all repository cache, and a build will need to re-download the LLVM toolchain.

### Module testing failures

If you are running in docker, there is a known issue with valgrind. Before running any tests you must set your ulimit down from the default of 1073741804 to something more reasonable. Since you cannot pass `--ulimit` to the docker run args of a Bazel sandbox, you must set this globally in your docker daemon. Thankfully, GitHub appears to already do this. But for a local install you will need to add the following to the `daemon.json` file and restart docker.

```
{
  "default-ulimits": {
    "nofile": {
      "Name": "nofile",
      "Soft": 65536
      "Hard": 65536,
    }
  }
}
```

### Testing the full distribution of packages

There is no straightforward way of running a full test suite across all package imports. Bazel doesn't support to support a wildcard expansion for test targets that span multiple modules. For this reason we have a [Distribution File](distribution.txt) and supporting rule in our `.bazelrc` file that enables you to run all tests across all repos in the following way:

```
bazel test --config=distribution
```