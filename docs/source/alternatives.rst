Alternatives
============

The ROS Central Registry is not the first Bazel build system for ROS. However, it offers the highest degree of test coverage, supports the widest range of messaging middleware implementations, and is the first to declare interface dependencies at a sub-package granularity. From ROS "Lyrical" onwards, we endeavor to follow each upstream distribution release with a corresponding Bazel release.

Our implementation would not be possible without the work of others in the ROS community. Notably, we use many ideas from the IDL aspects from `mvukov/rules_ros2 <https://github.com/mvukov/rules_ros2>`__ and the Bazel module encapsulation from `idealworks/bazel-public-registry <https://github.com/idealworks/bazel-public-registry>`__.

We are aware of many other proprietary and open source approaches. In the remainder of this page we describe our best understanding of each of the well-adopted open source projects. We do this so that readers can understand how our approach is positioned with respect to other approaches, so that they can make an informed decision about which approach is best for their own needs.

`mvukov/rules_ros2 <https://github.com/mvukov/rules_ros2>`__
------------------------------------------------------------
This approach follows the Bazel build-everything-from-source philosophy by providing a collection of ``BUILD.bazel`` files for many existing packages. The tooling downloads the ``humble/2024-12-05`` release of the ``ros2.repos`` file, which it transforms into a structured list of package definitions, which call a Bazel rule to register the package with a source download location, collection of patches, and Bazel ``BUILD`` file. Supports only ROS 2 Humble with ConnextDDS as messaging middleware.

Key features of this project are:
    * Builds C++, Python and Rust messages from msg, srv, action files.
    * Offers CycloneDDS as messaging middleware.
    * Does not include PyTest or GTest targets in the ``BUILD.bazel`` files provided for the foundational ROS packages. This means that ``bazel test`` cannot be used to test functionality, and so modification and testing of packages must go through the traditional ci.ros.org pipeline.
    * No process in place for moving the package patches upstream.
    * This project also includes Bazel targets for common ROS 2 commandline tools, like ``ros2 topic``

In June 2025 the entire project was packaged up as one single Bazel module: https://registry.bazel.build/modules/com_github_mvukov_rules_ros2  

`ApexAI/rules_ros <https://github.com/ApexAI/rules_ros>`__
----------------------------------------------------------
Similar to the first approach, this one also builds everything from source. Their BUILD files are stored in a directory hierarchy. This project appears to be more of a skeleton work in progress than something that can be used immediately. This is because some fairly critical packages, like ``sensor_msgs``, don’t seem to be included in their implementation. It supports only ROS "Humble", and is tested only on Ubuntu 20.04. It does IDL generation, builds C++ targets only.

`idealworks/bazel-public-registry <https://github.com/idealworks/bazel-public-registry>`__
------------------------------------------------------------------------------------------
Appears to be the only implementation of a ROS distribution as Bazel modules through a custom registry. Packages in the ROS ecosystem are prefixed with ros2. Based on the naming structure, it appears to be pinned to ROS 2 Jazzy and uses a lot of the Bazel providers and aspects from the module `mvukov/rules_ros2 <https://github.com/mvukov/rules_ros2>`__. Unfortunately, there is no supporting tooling around module maintenance, and so it unclear how or when modules will be updated in response to a new upstream ROS release.

`RobotLocomotion/drake-ros <https://github.com/RobotLocomotion/drake-ros>`__
----------------------------------------------------------------------------
Instead of building everything from source, the project exposes an existing ROS workspace -- either on a local machine or from some archive available online. Their implementation scrapes the workspace to create Bazel labels for all targets. In a sense, this project seeks to bridge a Bazel workspace to a ROS workspace. Since the scraping is done during the analysis phase, it can introduce some latency that causes ``bazel`` commands to take longer than expected to complete. Also, since the upstream products are pre-compiled, you can't for example easily add address, memory or thread sanitizers to your build. They also include tooling to enforce network isolation during tests, which helps make parallel test execution more resilient to interference.

