<p align="center">
<img width="256" src="docs/source/rcr_logo.png"/>
<br/>   
<font size="6">ROS Central Registry</font>
<br/>   
<font size="2">ROS Packages as Bazel Modules</font>
<br/>   
<br/>   
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/ubuntu-24.04_bazel-8.5.1_x86_64.json">
&nbsp;
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/ubuntu-24.04_bazel-8.5.1_aarch64.json">
<br/>   
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/ubuntu-24.04_bazel-9.0.0_x86_64.json">
&nbsp;
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/ubuntu-24.04_bazel-9.0.0_aarch64.json">
<br/>
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/documentation.json">
<br/> 
<img src="https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/asymingt/4d05b2382879fbd51f9a07bb7c63fb29/raw/linting.json">
<br/>
</p>

<hr/>

The [ROS Central Registry](http://intrinsic-opensource.github.io/ros-central-registry) provides [Bazel](https://bazel.build) modules for [Robot Operating System (ROS)](https://ros.org) core packages. Our philosophy is to build everything from source, using the dependency management system provided by the [Bazel](https://bazel.build) ecosystem to ensure consistency across ROS releases. A lot of the work in this project is inspired by Milan Vukov's [rules_ros2](https://github.com/mvukov/rules_ros2) project, one of [several existing Bazel build systems for ROS](https://github.com/RobotLocomotion/drake-ros/blob/main/bazel_ros2_rules/lib/README.md). 

> [!WARNING]
> This repository is a proof of concept and under active open development, and so no guarantees are made about stability. Please do not depend on this code until we have an official first release!

To get started, make sure that you recurse submodules when you clone this repository:

```python
git clone https://github.com/intrinsic-opensource/ros-central-registry.git --recurse-submodules
```

We currently support C, C++ and Python client nodes, as well as the generation of ROS messages, services and actions from message definition chains. Please take a look at the README in the `examples` directory for a quickstart guide.
