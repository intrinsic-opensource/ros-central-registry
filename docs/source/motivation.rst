Why?
====

Bazel is an open-source build tool that originated within Google. It supports many different programming languages, and its goal is to build gigantic projects reproducibly and efficiently. 

By enforcing the compilation of all code **from source** within a **strictly sandboxed environment**, Bazel eliminates the classic `"it works on my machine"` issues caused by operating system and dependency discrepancies between code on different machines. 

This cohesive paradigm replaces the fragile, environment-dependent workspace model with a rigorous dependency graph, unlocking advanced capabilities like **build and test caching**, as well as **remote execution** to drastically **accelerate build times** while ensuring that tests and binaries are **reproducible** across any developer, production or CI environment.

Traditionally, Bazel has been used in large monorepos, but with the introduction of **Bazel modules** it has become considerably easier to use Bazel in smaller, more distributed projects. This makes Bazel an appealing alternative to the existing ROS build system.

The ROS Central Registry is analagous to the Bazel Central Registry. It serves as a central registry for hosting ROS packages as Bazel modules. Adding support for a new ROS package is as simple as following our developer guidelines and openeing up a pull request to this repository.