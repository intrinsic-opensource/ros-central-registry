.. _developer_guide:

Developer guide
===============

Overview
++++++++

ROS is a federated ecosystem of packages, with no central authority controlling the versioning or release of individual packages. However, the ROS Team maintains several **distributions** of ROS packages that are tied to specific operating systems. And, periodically they release updated package version collections for a distribution, which is called a **release**.

Releases are made via the `rosdistro <https://github.com/ros/rosdistro>`__ repository. The open source community opens pull requests against this repository to update package versions. Every month or so a ROS maintainer picks a specific and well-tested commit in this repository to tag with a release version. For example the `kilted/2025-12-12 <https://github.com/ros/rosdistro/tree/kilted/2025-12-12>`__ tag is a release of the **kilted** distribution on 20212-12-12.

The ROS Central Registry lags behind rosdistro releases, because developers introduce breaking changes, and we will occasionally need to patch upstream source code to make the collection of packages build and pass tests in Bazel.

The rough process that we follow is:
    (1) Bootstrap a new release as a copy of the previous one.
    (2) Fix any generated modules that have build or test failures.
    (3) Tag a final release with a version number.
    (4) Patch any bugs that are identified after the release.

Bootstrapping
+++++++++++++

Our Bazel release process begins by running the following command to -- for example -- generate a new release for the **rolling** distribution using the **2026-01-21** release tag.

.. code-block:: shell

   bazel run //tools:bootstrap -- --old rolling/2025-12-04 --new rolling/2026-01-21

Under the hood this command uses `superflore <https://github.com/ros-infrastructure/superflore>`__ to transform package version and dependency information from the rosdistro repository into a collection of Bazel modules in teh ``modules`` folder. Notably, it generates a ``MODULE.bazel`` for each package in the release, along with some additional build files. It does this using these three rules:
    1. If a package version has not changed, it does nothing.
    2. If a package is present in the old release, it copies the ``source.json``, patches and overlays from the previous version, updating only the upstream tarball and integrity hash to the new version.
    3. If a package is present in the new release but not in the old release, it creates a ``source.json`` with the upstream tarball and integrity hash but nothing else.

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

Firstly, you need to navigate to the release directory containing the module you want to patch, eg.

.. code-block:: text

   cd releases/rolling/2026-01-21

Next, you use the bazel vendor command to generate the vendor directory for the module. This command essentially follows the module's source.json to download the upstream source code, apply any patches, and generate the vendor directory.

.. code-block:: text

   bazel vendor --repo=@rclcpp --vendor_dir=vendor

This will create a vendor directory with a copy of the upstream source code, applied patches, and any overlays.

::

    ros-central-registry/
        ├── ...
        └── releases/
            ├── ...
            ├── rolling/
            │   ├── ...
            │   ├── 2026-01-21/
            │   │   ├── vendor/
            │   │   │   ├── ...
            │   │   │   ├── rclcpp+/            # This is where you make your changes.
            │   │   │   ├── @rclcpp+.marker     # This is used to identify that this is a vendored repository.
            │   │   │   └── VENDOR.bazel        # This is used to configure how external repositories are handled in vendor mode.
            │   │   ├── .bazelrc
            │   │   ├── .bazelversion
            │   │   └── MODULE.bazel
            │   └── ...
            └── ...

Then, you must build and test with the ``--vendor_dir`` directory pointing to the vendor directory.

.. code-block:: text
    
    bazel build --vendor_dir=vendor @rclcpp//...
    bazel test  --vendor_dir=vendor @rclcpp//...

You can now iterate and test changes to the source code in the ``rclcpp+/`` directory. You will be doing things like patching source code and adding ``BUILD.bazel`` files until you get the desired result. We have a few tools to help you with this process, which we'll describe in the following section.

Once you have a working patch, you can use the ``patch`` tool to generate a patch for this release:

.. code-block:: text

   bazel run //tools:patch -- --release=rolling/2026-01-21 --repo=rclcpp

What this will do is vendor a bare ``rclcpp`` module (without patches or overlays) into a separate directory and calculate the diff between the vanilla module and the changes that you made. It will then transform these changes into a set of patches and overlays, and automatically create a new package version with an incremented patch number.

::

    ros-central-registry/
    ├── ...
    ├── modules/
    │   ├── ...
    │   ├── rclcpp/
    │   │   ├── ...
    │   │   ├── rolling.30.1.4-1/                   # original version
    │   │   │   ├── overlay/
    │   │   │   |   └── BUILD.bazel
    │   │   │   ├── patches/
    │   │   │   |   ├── 0001-fix-build-error.patch
    │   │   │   |   └── 0002-fix-test-failure.patch
    │   │   │   ├── MODULE.bazel
    │   │   │   └── source.json
    │   │   ├── rolling.30.1.4-1.rcr.1/             # patch version
    │   │   │   ├── overlay/
    │   │   │   │   ├── tests/
    │   │   │   │   │   └── BUILD.bazel             # new tests added by you
    │   │   │   │   └── BUILD.bazel                 # new build files added by you
    │   │   │   ├── patches/
    │   │   │   |   ├── 0001-fix-build-error.patch
    │   │   │   |   ├── 0002-fix-build-error.patch
    │   │   │   |   └── 0003-additionalfixes.patch  # new fixes to source code
    │   │   │   ├── MODULE.bazel
    │   │   │   └── source.json                     # updated patches, overlays and hashes
    │   │   └── metadata.json
    │   └── ...
    └── releases/
        ├── ...
        ├── rolling/
        │   ├── ...
        │   └── 2026-01-21/                         # new release
        │       ├── .bazelrc
        │       ├── .bazelversion
        │       └── MODULE.bazel                    # updated to reference your patch version
        └── ...


You are now in a position to set up a PR against the ROS Central Registry, adding the new module version (eg. folder ``ros-central-registry/modules/rclcpp/rolling.30.1.4-1.rcr.1``) and the updated release (eg. ``ros-central-registry/releases/rolling/2026-01-21/MODULE.bazel``). Pull requests that anything other than these changes will be rejected.

Tooling
+++++++

We know that writing Bazel modules can be a bit laborious, and so we have a few tools to help you with this process. The first of these tools helps with ``BUILD.bazel`` files for interfaces.  What it does is scan your source tree for ``*.action``, ``*.msg`` and ``*.srv`` files. It then parses these files to work out the dependencies -- both between interfaces in your package and between your package and other packages. Using this it will automatically write the ``BUILD.bazel`` files for your interfaces.

As an example, let's say that we want to auto-generate the ``BUILD.bazel`` files for the newly-added ``px4_msgs`` package. To do this, we can run the following command during patching:

.. code-block:: text

   bazel run //tools:interfaces -- vendor/px4_msgs+

This will create the ``vendor/px4_msgs+/msg/BUILD.bazel`` and ``vendor/px4_msgs+/srv/BUILD.bazel`` files for you. It is assumed that the ``MODULE.bazel`` file already contains ``bazel_dep(...)``` calls for third party message dependency from bootstrapping.

