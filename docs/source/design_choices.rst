Design choices
==============

This page is targeted at those who are interested in learning more about various design decisions that the team needed to make when developing the ROS Central Registry.

Versioning
----------

Let's say we are creating a new release for ``kilted/2026-01-22`` release tag. In this release the ``rclcpp`` package moved from ``29.5.5-1`` to ``29.5.6-1``. Bootstrapping will result in a new Bazel module for rclcpp with the version ``kilted.29.5.6-1``. Assuming we have to fix some build erros as a result of the code change, the released version will be ``kilted.29.5.6-1.bcr.1``, with ``.bcr.1`` signalling that a patch was needed, and this will be tagged in the first release of package ``rosdistro`` with version ``kilted.2026.01.22.bcr.1``.


.. code-block:: shell

    multiple_version_override(
        module_name = "sensor_msgs",
        versions = [
            "jazzy.29.5.6-1.bcr.1",
            "kilted.29.5.6-1.bcr.1",
        ],
    )

.. code-block:: shell

    multiple_version_override(
        module_name = "sensor_msgs",
        versions = [
            "jazzy.29.5.6-1.bcr.1",
            "kilted.29.5.6-1.bcr.1",
        ],
    )

Interfaces
----------
The ROS Central Registry adopts many of the aspects of `rules_ros2 <https://github.com/mvukov/rules_ros2>`__ with a few key differences. The most obvious one is that interface dependencies are declared at the message level, not the package level. As an example, consider adding the following new message ``ExampleMessage.msg`` that depends on two common interfaces, ``sensor_msgs/CompressedImage`` and ``std_msgs/String``.

.. code-block:: python

    std_msgs/String message
    sensor_msgs/CompressedImage back_left
    sensor_msgs/CompressedImage back_right
    sensor_msgs/CompressedImage front_left
    sensor_msgs/CompressedImage front_right

When you create a Bazel target for this interface, you must declare the dependencies at the message level, and not the package level. This allows the IDL generators -- which convert this language-neutral interface description into language-specific code -- to generate exactly what is needed for the target, without any extra code that is not used. This makes for faster builds and smaller build products.

.. code-block:: python

    load("@ros//:rules.bzl", "ros_interface")

    ros_interface(
        name = "ExampleMessage",
        src = "ExampleMessage.msg",
        deps = [
            "@sensor_msgs//msg:CompressedImage",
            "@std_msgs//msg:String",
        ],
    )

Other differences are that we have -- as best as possible -- copied the rule semantics from protocol buffers (for example ``cc_ros_library`` is the C++ bindings for ROS interface in the same way ``cc_proto_library`` is the C++ bindings for protocol buffer messages) and used bare language rules (``cc_binary`` in stead of ``ros_cc_binary``) to avoid any opaqueness in our design.

.. code-block:: python

    load("@rosidl_generator_cpp//:rules.bzl", "cc_ros_library")
    load("@rules_cc//cc:defs.bzl", "cc_binary")

    cc_ros_library(
        name = "example_ros_cc_msgs",
        deps = [
            "//msg:ExampleMessage",
        ],
    )

    cc_binary(
        name = "example_ros_publisher_cc",
        srcs = ["ros/example_ros_publisher.cc"],
        deps = [
            "@rclcpp",
            ":example_ros_cc_msgs"
        ],
    )

Shared libraries
----------------

Before the `cc_shared_library`` rule was introduced, shared libraries had to be built using ``cc_binary`` with the ``linkshared = True`` attribute. This was not ideal because it meant that the shared library would be built as a binary, which is not what we want. It also meant that we had to manually link the shared library to the binary, which was not ideal.

Runtime data
------------

One of the key differences between a CMake-based build and a Bazel-based build for ROS is how runtime data is handled. In a CMake build, runtime data is typically installed to a fixed location (for example to the ``install`` folder in a ROS workspace) with an ``INSTALL`` function. When you source ``install/setup.bash`` it automatically adds the ``install`` folder to your ``AMENT_PREFIX_PATH`` environment variable. Each client library then relies on a language-specific ``ament_index`` package to provide something like a ``get_package_share_directory(package_name)`` function to access data.

The issue with the CMake approach is that unrelated packages (not just dependencies and the current package) can install data into the same directory. This can cause conflicts and couples the behavior of the node to an execution context (it not hermetic) which can in turn cause nondeterministic failures.

Bazel resolves this problem using ``runfiles``. What this looks like in practice is that whenever a target is built, every data dependency it needs is listed in its ``data`` attribute. This ``data`` attribute is then used to create a ``runfiles`` tree, which is essentially a set of symbolic links to the underlying data fails. This makes the build hermetic and prevents conflicts between unrelated packages.

To avoid having to change every all code in ROS to understand Bazel runfiles, we have modifed the ``ament_index`` package for each language to treat the root of the runfiles folder as the prefix path if the ``AMENT_PREFIX_PATH`` environment variable is unset. For some ``example_project`` we encapsulate data with a ``ros_data`` rule, which serves to recreate the ament index folder layout within the runfiles tree of a specific binary. 

.. code-block:: python

    load("@ros//:rules.bzl", "ros_data")
    load("@rules_cc//cc:defs.bzl", "cc_library")

    cc_library(
        name = "util_library",
        hdrs = ["include/example_project/util.hh"],
        srcs = ["src/util.cc"],
        includes = ["include"],
    )

    ros_data(
        name = "data",
        data = [
            "config/default.yaml",
            ":util_library",
            "@gtsam//:gtsam",
        ],
    )

    cc_binary(
        name = "example_ros_publisher_cc",
        srcs = ["ros/example_ros_publisher.cc"],
        data = [
            ":data",
            "@ament_index_cpp"
        ],
    )

When you run ``bazel run //:example_ros_publisher_cc`` the ``example_ros_publisher_cc.runfiles`` directory is created, which organizes the libraries and binaries into a common ``lib`` folder, and the headers and shared data into ``include`` and ``share`` folders respectively, organized by package.

.. code-block:: python

    bazel-bin/
        example_ros_publisher_cc
        example_ros_publisher_cc.runfiles/
            include/
                example_project/util.hh
            lib/
                libutil_library.so
                gtsam
            share/
                example_project/
                    config/
                        default.yaml

This illustrates the principle. In practice the rule is more complex because it also has to traverse the dependency chain to bring in files for the teansitive dependencies of the includes.
