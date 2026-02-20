Alternatives
============

The ROS Central Registry is not the first Bazel build system for ROS. However, it offers the highest degree of test coverage, supports the widest range of messaging middleware implementations, and is the first to declare interface dependencies at a sub-package granularity. From ROS "Lyrical" onwards, we endeavor to follow each upstream distribution release with a corresponding Bazel release.

Our implementation would not be possible without the incredible work of others in the ROS community. It borrows many concepts from other projects. Notably, the IDL aspects from `mvukov/rules_ros2 <https://github.com/mvukov/rules_ros2>`__ and the Bazel module encapsulation from `idealworks/bazel-public-registry <https://github.com/idealworks/bazel-public-registry>`__.

We are aware of many other propreitary and open source approaches. In the remainder of this page we describe our best understanding of each of the well-adopted open source projects. We do this so that readers can understand how our approach is positioned with respect to other approaches, so that they can make an informed decision about which approach is best for their own needs.

mvukov/rules_ros2
-----------------
This approach follows the Bazel build-everything-from-source philosophy by providing a collection of ``BUILD.bazel`` files for many existing packages. The tooling downloads the `humble/2024-12-05` release of the ``ros2.repos`` file, which it transforms into a structured list of package definitions, which call a Bazel rule to register the package with a source download location, collection of patches, and Bazel ``BUILD`` file. Supports only ROS 2 Humble with ConnextDDS as messaging middleware.
Builds  C++, Python and Rust targets, parses msg/srv/action files.
Does not include PyTest or GTest targets in the ``BUILD.bazel`` files provided for the foundational ROS packages. This means that ``bazel test …`` cannot be used to test functionality, and so modification and testing of packages must go through the traditional CMake + ci.ros2.org pipeline.
No process in place for moving the package patches upstream.
Their solution to the message generation is through the Bazel ``ros2_interface_library`` rule that provides an aspect (code that traverses the action graph but doesn’t modify it) to resolve cross-package message dependencies, and produce language-specific targets. 
This project also includes Bazel targets for common ROS 2 commandline tools, like “ros2 topic”. However, we should think carefully whether these runtime tools need to be part of a build system.
In June 2025 the entire project was packaged up as one single Bazel module: https://registry.bazel.build/modules/com_github_mvukov_rules_ros2  

idealworks/bazel-public-registry
--------------------------------
Appears to be the only bzlmod implementation of a ROS 2 system, provided as a custom registry. Packages in the ROS ecosystem are prefixed with ros2. Based on the naming structure, it appears to be pinned to ROS 2 Jazzy and uses a lot of the Bazel providers and aspects from the module  mvukov’s implementation above. It feels like more or an augmentation to mvukov’s rules, where new packages are added as independent bazel modules.

ApexAI/rules_ros
----------------
Similar to the first approach, this one also builds everything from source. Their BUILD files are stored in a directory hierarchy. This project appears to be more of a skeleton work in progress than something that can be used immediately. This is because some fairly critical packages, like sensor_msgs and control_msgs, don’t seem to be supported by their implementation.
Supports only ROS 2 Humble, tested only on Ubuntu 20.04
Builds C++ targets only, parses msg/src/action/idl files.
There is an impractically small set of contributed BUILD files, compared to mvukov/rules_ros2.

RobotLocomotion/drake-ros
-------------------------
This approach differs from the previous two because instead of building everything from source, the authors provide Bazel targets that wrap Python tools that scrape CMake exports from an existing workspace to organize them in a way that makes sense to Bazel. It then automatically creates targets for the files, libraries, executables, messages, for each package which is prefixed by the ROS 2  package name to keep organized.
Nominally supports  all middleware offered by the underlay workspace.
Builds C++ and Python targets, parses msg/src/action/idl files.
Since this approach works off a pre-built underlay, it is in principle possible to build against ROS 2 workspace, including packaged releases or your own merge or symlink installed workspace.
The scraping is done by Python scripts invoked by the Bazel rule ros2_local_repository  in the  analysis phase, and this can be slow for large workspaces. Under the hood this uses the cmake API to find the targets offered by a package, but since a pre-compiled workspace is used no consideration is made for the relationship between package source code and targets.
It cannot gracefully handle cases when a package installs products  to a location outside of its own names output folder. This pattern is atypical for ROS, but it does happen in select packages.
They include tooling that enforces network isolation for the messaging middleware, which prevents. I suspect this is to handle the case where Bazel runs multiple tests in parallel, causing the test fixtures to destructively interfere with each other. This is probably similar to the isolation performed by ament_cmake_ros that uses coordination through shared sockets to increment the ROS domain ID to ensure that ROS nodes from different tests in parallel cannot communicate with each other.
Their solution to message generation is to reimplement the parsing logic in rosidl.bzl and expose a rosidl_interfaces_group Bazel function  for transforming a set of message definitions to Bazel targets, one for C++ and Python targets for dependent targets that use these messages.