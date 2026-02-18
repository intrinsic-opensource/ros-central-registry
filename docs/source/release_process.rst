Release process
===============

Overview
++++++++

ROS is a federated ecosystem of packages, with no central authority controlling the versioning or release of individual packages. However, the ROS Team maintains several **distributions** of ROS packages that are tied to specific Operating Systems. And, periodically they release updated packages version collections for a distribution, which is called a **release**.

Releases are made via the `rosdistro <https://github.com/ros/rosdistro>`__ repository. Contributors will typically make commits against this repository to update package versions for each distribution. Every month or so a ROS maintainer picks a specific and well-tested commit in this repository to tag with a release version. For example the `kilted/2025-12-12 <https://github.com/ros/rosdistro/tree/kilted/2025-12-12>`__ tag is a release of the **kilted** distribution on 20212-12-12.

The ROS Central Registry lags behind rosdistro releases, because we will occasionally need to patch upstream source code to make the collection of packages build and pass tests in Bazel. The rough process that we follow is:
    (1) Bootstrap a new release as a copy of the previous one.
    (2) Fix any generated modules that have build or test failures.
    (3) Tag a final release with a version number.
    (4) Patch any bugs that are identified after the release.

Bootstrapping
+++++++++++++

Our Bazel release process begins by running the following `superflore <https://github.com/ros-infrastructure/superflore>`__ command to, for example, generate a new release for the **rolling** distribution using the **2026-01-21** release tag. The command assumes that the `ros-central-registry <https://github.com/intrinsic-opensource/ros-central-registry>`__ repository is already checked out at `/path/to/ros-central-registry`.

.. code-block:: shell

   superflore-gen-bazel --ros-distro rolling --ros-release 2026-01-21 \
        --output-repository-path /path/to/ros-central-registry

This command transforms package dependency information from the rosdistro repository into Bazel modules within the `modules` folder of the ros-central-registry repository. To understand the specifics, you need to know a little about how Bazel modules work. 

Patching
++++++++