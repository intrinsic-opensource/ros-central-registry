Quick start
===========
The purpose of this page is to get you up and running with Bazel and ROS as quickly as possible. For those who've used ``colcon`` to build ROS from source, we've also added a tab to commands to help you mentally map concepts from known ``colcon`` commands to their Bazel equivalents.

Setup your workspace
--------------------

Install Bazel
+++++++++++++

The easiest way to get started is to download and install ``bazelisk``, which you can get from `this page <https://bazel.build/install>`__. If you uncomfortable with a tool that downloads and runs arbitrary code from the internet, you can install Bazel using your system's package manager. Before doing so, please consult the `official documentation <https://docs.bazel.build/versions/stable/install.html>`__ to ensure you install a version that is compatible with this registry.

Copy release
++++++++++++

The next step is to setup your Bazel workspace. The easiest way to go about doing this is to create a new folder, for example ``example_ros_workspace``,  and copy the three files from the desired release in our `releases folder <https://github.com/intrinsic-opensource/ros-central-registry/tree/main/releases>`__ into that folder. You should then have a folder that looks like this:

::

    example_ros_workspace/
    ├── .bazelrc
    ├── .bazelversion
    └── MODULE.bazel

The ``.bazelrc`` file is used to configure Bazel for your project. The ``.bazelversion`` file is used to specify the version of Bazel to use. The ``MODULE.bazel`` file is used to specify the dependencies for the project, which in our case is the entire ROS ecosystem for the release you have chosen.

One of the dependencies is the ``llvm_toolchain``, which is essentially a C++ toolchain that is used to build the ROS packages. The advantage of using our toolchain over a default provided by your operating system is that it ensures consistent build results across different platforms.

.. note::

   Note that the ``.bazelrc`` file points to a public, read-only build cache offered by Intrinsic. This means that you can benefit from the fact that a CI system has already built the same ROS packages for your architecture. This means you can download prebuilt binaries instead of having to build them yourself. This can significantly speed up build times, especially for large projects. If you don't want to use this cache, you can disable it in the ``.bazelrc`` file.

Bazel fundamentals
------------------

Targets
+++++++

1. The ampersand ``@`` is a prefix that means "this is a Bazel module".
2. Two forward slashes ``//`` means relative to the root of the current module. 
3. The elipses ``...`` is a wildcard that means "everything in this package". So ``@rclcpp//...`` means "everything in the ``@rclcpp`` package".
4. ``@rclcpp`` is a short-form target. Bazel automatically expands this for your it to its long-form canonical target name ``@rclcpp//:rclcpp``.
5. If you specify no module before a root, it assume you mean the current workspace. So ``//...`` means "everything in this workspace".

So, now let's pretend that we have added a C++ library called ``foo`` to our workspace. We then have a ROS node called ``bar`` that uses this library. Our new folder structure might look something like this:

::

    example_ros_workspace/
    ├── docs/
    │   └── README.md
    ├── libraries/
    │   └── foo/
    │       ├── include/
    │       │   └── foo/
    │       │       └── foo.hh
    │       ├── src/
    │       │   └── foo.cc
    │       ├── config.yaml
    │       └── BUILD.bazel
    ├── nodes/
    │   ├── bar.cc
    │   └── BUILD.bazel
    ├── .bazelrc
    ├── .bazelversion
    ├── BUILD.bazel
    └── MODULE.bazel

Now, let's say we want to build the ``foo`` library. We can do this with any of following commands.
      
.. code-block::

   bazel build //libraries/...
   bazel build //libraries/foo/...
   bazel build //libraries/foo:foo
   bazel build //libraries/foo

The important thing to note is that in order for the colcon ``:`` separator to work, you must have a ``BUILD.bazel`` file in the target directory. For this reason, in the example above ``bazel build //docs/...`` will not work because there is no ``BUILD.bazel`` file in the ``docs`` subdirectory.

Build rules
+++++++++++

Continuing with the hypothetical example above, let's take a look at the ``BUILD.bazel`` file in the ``nodes`` subdirectory. The ``cc_binary`` rule is used to build an executable C or C++ program. The ``srcs`` attribute is a list of source files to compile, and the ``deps`` attribute is a list of dependencies to link against. The ``data`` attribute is a list of data files that are needed by the executable at runtime.

.. code-block::
   :caption: nodes/BUILD.bazel

   load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_test")

   package(default_visibility = ["//visibility:public"])

   cc_binary(
      name = "bar",
      srcs = ["bar.cc"],
      data = ["config.yaml"],
      deps = [
         "//libraries/foo",  # expands to //libraries/foo:foo
         "@rclcpp",          # expands to @rclcpp//:rclcpp
      ],
   )

The important thing to note is that the ``deps`` attribute is a list of dependencies to link against. So, in this case, ``bar`` depends on ``foo`` and ``rclcpp``. Under the hood, Bazel analyzes the dependency chain and builds something called the "action graph". This is a directed acyclic graph (DAG) of all the actions that need to be performed to build the target. Using this is can schedule the build in parallel, and can also cache intermediate results. This is what makes Bazel so fast. 

.. tip::

   Bazel has a very powerful query engine. For example, you can run ``bazel query //...`` to see all the targets in the workspace, or ``bazel query "deps(//nodes:bar)"`` to see the dependencies of a specific target ``bar``. This makes it very easy to understand what targets are available, as well as the dependency structure of your workspace.

Build products
++++++++++++++

All intermediary and final build outputs are cached to the ``bazel-out/`` directory in the root of the active workspace. You will spend more time looking at the contents of the ``bazel-bin/`` subdirectory, which contains symbolic links to subdirectories in ``bazel-out/``. The last directory is ``bazel-testlogs/``, which contains the logs of all the tests that have been run. All of these directories are created automatically.
::

    example_ros_workspace/
    ├── bazel-bin/
    │   ├── ...
    │   └── nodes/
    │       └── bar/
    │           ├── bar                               # the executable you built
    │           └── bar.runfiles/                     # all data needed at runtime 
    │               ├── _main/                        # runtime data described on bar
    │               │   └── _solib_{k8,arm64,...}/.   # mangled bazel libraries
    │               │   └── config.yaml               # symlink to config.yaml in nodes
    │               └── rclcpp+/                      # runfile data from other modules
    │               └── ...
    ├── docs/
    │   └── README.md
    ├── libraries/
    │   └── foo/
    │       ├── include/
    │       │   └── foo/
    │       │       └── foo.hh
    │       ├── src/
    │       │   └── foo.cc
    │       ├── data/
    │       │   └── config.yaml
    │       └── BUILD.bazel
    ├── nodes/
    │   ├── bar.cc
    │   └── BUILD.bazel
    ├── .bazelrc
    ├── .bazelversion
    ├── BUILD.bazel
    └── MODULE.bazel

If you do need to clean the cache, you can do so with the following command.

.. code-block::

   bazel clean --expunge


ROS packages
------------

Build packages
++++++++++++++

All C++ nodes use ``rclcpp``, which is the C++ client library for ROS. So it makes sense to build this package first. Change directory into the ``example_ros_workspace`` folder and run the following command. 

.. tabs::

   .. group-tab:: Bazel
            
      .. code-block::

         bazel build @rclcpp

   .. group-tab:: Colcon

      .. code-block::

         colcon build --packages-up-to rclcpp

In Bazel we call ``@rclcpp`` a short-hand target. The ampersand ``@`` is a special prefix that means "this is a Bazel module". Bazel automatically expands it to its default canonical target name ``@rclcpp//:rclcpp``. What this means is build target ``:rclcpp`` in package ``@rclcpp``. Some packages have more than one target. For example, ``@rclcpp`` has another target ``@rclcpp//:type_adapter``. Try building that one  in stead with ``bazel build @rclcpp//:type_adapter`` and see what happens.

Test packages
+++++++++++++

When we migrate packages to Bazel we also migrate their tests. To run the tests for an entire ROS package, you can use the following command. Note that we use the ``--jobs 1`` flag to run the tests sequentially. We do this because we don't know how much CPU or RAM your machine has, and we don't want to overwhelm it. Also, some of the ``rclcpp`` tests use the network, and if we run two in parallel without proper resource reservation, they might conflict with each other.

.. tabs::

   .. group-tab:: Bazel
            
      .. code-block::

         bazel test --jobs 1 @rclcpp//...

   .. group-tab:: Colcon

      .. code-block::

         colcon test --executor sequential --packages-select rclcpp

ROS fundamentals
----------------

Interfaces
++++++++++

One of the fundamental advantages of ROS is provides a standardized way of exchanging and storing information. This allows for lots of reusable tooling and easy interoperation between packages, even if they are written in different programming languages. What drives this concept is the idea of an ``interface``, which is an agreed-upon strategy for exchanging information.

The Bazel rules for interfaces are inspired by the Bazel rules for protocol buffers. In essence, you define a ROS interface with the ``ros_interface`` rule. If you re-use other packages' interfaces, you add them to the ``deps`` attribute of the ``ros_interface`` rule. In so doing you construct a graph of interfaces. The language_specific rules, eg ``c_ros_library``, ``cc_ros_library``, ``py_ros_library`` are then called on a collection of interfaces to generate the language-specific interfaces.

So, let's tak a look at a hypothetical ``Example.msg`` file containing three fields. This example message imports and uses message types from two other packages, ``std_msgs`` and ``sensor_msgs``.

.. code-block:: text
   :caption: Example.msg

   std_msgs/String message
   sensor_msgs/CompressedImage back_left
   bool is_enabled

If one wanted to define this interface in a ``BUILD.bazel`` file, one would do so as follows. This presumes of course that the ``sensor_msgs`` and ``std_msgs`` packages have been added to the workspace. This will be automatically the case if you are using a root ``MODULE.bazel`` file from one of our releases.

.. code-block::

   load("@ros//:interfaces.bzl", "ros_interface")

   ros_interface(
      name = "example_msg",
      src = "Example.msg",
      deps = [
         "@sensor_msgs//msg:CompressedImage",
         "@std_msgs//msg:String",
      ],
   )

If you have prior experience building a ROS package, you might be surprised that dependency is now described at the message, not the package level. This is a concious design choice to allow for more fine-grained dependency and faster build times. At the point where you want to use this interface in a C++ node, you would need to transform this (and perhaps combine with other messages) into a ``cc_ros_library`` target. Here is what that looks like:


.. tabs::

   .. group-tab:: C
            
      .. code-block::
         
         load("@rosidl_generator_c//:rules.bzl", "c_ros_library")

         c_ros_library(
            name = "msgs_c",
            deps = [
               ":example_msg",
               "@sensor_msgs//msg:Imu",
            ],
         )

   .. group-tab:: C++

      .. code-block::

         load("@rosidl_generator_cpp//:rules.bzl", "cc_ros_library")

         cc_ros_library(
            name = "msgs_cc",
            deps = [
               ":example_msg",
               "@sensor_msgs//msg:Imu",
            ],
         )

   .. group-tab:: Python

      .. code-block::

         load("@rosidl_generator_py//:rules.bzl", "py_ros_library")

         py_ros_library(
            name = "msgs_py",
            deps = [
               ":example_msg",
               "@sensor_msgs//msg:Imu",
            ],
         )

What's interesting about this example is that both ``@sensor_msgs//msg:Imu`` and ``@sensor_msgs//msg:CompressedImage`` depend on ``@std_msgs//msg:Header``. Bazel's action graph is aware of this, and will generate and build ``@std_msgs//msg:Header`` once, and the result will be re-used.

Shared data
+++++++++++


Example code
------------

