Developer guide
===============

Overview
++++++++

ROS is a federated ecosystem of packages, with no central authority controlling the versioning or release of individual packages. However, the ROS Team maintains several **distributions** of ROS packages that are tied to specific operating systems. And, periodically they release updated package version collections for a distribution, which is called a **release**.

Releases are made via the `rosdistro <https://github.com/ros/rosdistro>`__ repository. The open source community continually makes commits against this repository to update package versions for each distribution. Every month or so a ROS maintainer picks a specific and well-tested commit in this repository to tag with a release version. For example the `kilted/2025-12-12 <https://github.com/ros/rosdistro/tree/kilted/2025-12-12>`__ tag is a release of the **kilted** distribution on 20212-12-12.

The ROS Central Registry lags behind rosdistro releases, because developers introduce breaking changes, and we will occasionally need to patch upstream source code to make the collection of packages build and pass tests in Bazel.

The rough process that we follow is:
    (1) Bootstrap a new release as a copy of the previous one.
    (2) Fix any generated modules that have build or test failures.
    (3) Tag a final release with a version number.
    (4) Patch any bugs that are identified after the release.

Bootstrapping
+++++++++++++

Our Bazel release process begins by running the following `superflore <https://github.com/ros-infrastructure/superflore>`__ command to -- for example -- generate a new release for the **rolling** distribution using the **2026-01-21** release tag.

.. code-block:: shell

   bazel run //tools:bootstrap -- --old rolling/2025-12-04 --new rolling/2026-01-21

This command transforms package dependency information from the rosdistro repository into a set of versioned Bazel modules (ROS packages)within the `modules` folder of the ros-central-registry repository. This is an incredibly useful tool that saves us from managing these files by hand. It creates ``MODULE.bazel`` files for each package in the new release,  using two core principles:
    1. If a package is present in the old release, it copies the ``source.json``, patches and overlays from the previous version, updating only the upstream tarball and integrity hash to the new version.
    2. If a package is present in the new release but not the old one, it creates a ``source.json`` file for the new package, but leaves it emppty -- it does not create any patches or overlays.

So, continuing with our example, the ``bootstrap`` command will look at the new ``rolling/2026-01-21`` release tag and see that the ``rclcpp`` package has version ``rolling.30.1.4-1``. It will then look at the old ``rolling/2025-12-04`` release tag and see that the ``rclcpp`` package has version ``rolling.30.1.3-1``. After applying its logic, the folder layout will look something like this:

::

    ros-central-registry/
    ├── ...
    ├── modules/
    │   ├── ...
    │   ├── rclcpp/
    │   │   ├── ...
    │   │   ├── rolling.30.1.3-1/          # package version in rolling/2025-12-04 release
    │   │   │   ├── overlay/
    │   │   │   |   └── BUILD.bazel
    │   │   │   ├── patches/
    │   │   │   |   ├── 0001-fix-build-error.patch
    │   │   │   |   └── 0002-fix-test-failure.patch
    │   │   │   ├── MODULE.bazel
    │   │   │   └── source.json
    │   │   ├── rolling.30.1.4-1/          # package version in rolling/2026-01-21 release
    │   │   │   ├── overlay/
    │   │   │   |   └── BUILD.bazel
    │   │   │   ├── patches/
    │   │   │   |   ├── 0001-fix-build-error.patch
    │   │   │   |   └── 0002-fix-test-failure.patch
    │   │   │   ├── MODULE.bazel
    │   │   │   └── source.json
    │   │   └── metadata.json
    │   └── ...
    └── releases/
        ├── ...
        ├── rolling/
        │   ├── 2025-12-04/                # previous release
        │   │   ├── .bazelrc
        │   │   ├── .bazelversion
        │   │   └── MODULE.bazel
        │   ├── 2026-01-21/                # new release
        │   │   ├── .bazelrc
        │   │   ├── .bazelversion
        │   │   └── MODULE.bazel
        │   └── ...
        └── ...

Patching
++++++++


Release
+++++++

