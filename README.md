<p align="center">
<img width="256" src="docs/source/rcr_logo.png"/>
<br/>   
<font size="6">ROS Central Registry</font>
<br/>   
<font size="2">Developer preview</font>
<br/>
<br/>
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/documentation.json">
&nbsp; 
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/buildifier.json">
<br/>

|                       | <font size="2">FastDDS</font> | FastDDS Dynamic | CycloneDDS | Zenoh |
| :-------------------- | :----------: | :------------------: | :------------: |:--------: |
| ubuntu-24.04-x86_64   | <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-8.5.1_x86_64_rmw_fastrtps_cpp.json"/> <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-9.0.0_x86_64_rmw_fastrtps_cpp.json"/> | <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-8.5.1_x86_64_rmw_fastrtps_dynamic_cpp.json"/> <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-9.0.0_x86_64_rmw_fastrtps_dynamic_cpp.json"/> | <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-8.5.1_x86_64_rmw_cyclonedds_cpp.json"/> <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-9.0.0_x86_64_rmw_cyclonedds_cpp.json"/> | <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-8.5.1_x86_64_rmw_zenoh_cpp.json"/> <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-9.0.0_x86_64_rmw_zenoh_cpp.json"/> |
| ubuntu-24.04-arm64    | <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-8.5.1_aarch64_rmw_fastrtps_cpp.json"/> <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-9.0.0_aarch64_rmw_fastrtps_cpp.json"/> | <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-8.5.1_aarch64_rmw_fastrtps_dynamic_cpp.json"/> <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-9.0.0_aarch64_rmw_fastrtps_dynamic_cpp.json"/> | <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-8.5.1_aarch64_rmw_cyclonedds_cpp.json"/> <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-9.0.0_aarch64_rmw_cyclonedds_cpp.json"/> | <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-8.5.1_aarch64_rmw_zenoh_cpp.json"/> <img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/f796805a77e0f05cc837b18e60142773/raw/ubuntu-24.04_bazel-9.0.0_aarch64_rmw_zenoh_cpp.json"/>
</p>

The [ROS Central Registry](http://intrinsic-opensource.github.io/ros-central-registry) provides [Bazel](https://bazel.build) modules for [Robot Operating System (ROS)](https://ros.org) core packages. Our philosophy is to build everything from source, using the dependency management system provided by the [Bazel](https://bazel.build) ecosystem to ensure consistency across ROS releases. A lot of the work in this project is inspired by Milan Vukov's [rules_ros2](https://github.com/mvukov/rules_ros2) project, one of [several existing Bazel build systems for ROS](https://github.com/RobotLocomotion/drake-ros/blob/main/bazel_ros2_rules/lib/README.md). 

> [!WARNING]
> This repository is a proof of concept and under active open development, and so no guarantees are made about stability. Please do not depend on this code until we have an official first release!

Firstly, make sure that you have Bazelisk installed. You can find the latest version of Bazelisk [here](https://github.com/bazelbuild/bazelisk). Then, make sure that you recurse submodules when you clone this repository:

```python
git clone https://github.com/intrinsic-opensource/ros-central-registry.git --recurse-submodules
```

We currently support C, C++ and Python client nodes, as well as the generation of ROS messages, services and actions from message definition chains. Start by looking at the README in the `examples` folder. To get a list of all targets in the `examples` folder, run:

```
bazel query //examples/...
```