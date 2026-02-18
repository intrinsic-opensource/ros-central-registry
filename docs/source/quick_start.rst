Quick Start
===========

To use the ROS Central Registry in your Bazel project, add the following to your ``MODULE.bazel`` file:

.. code-block:: python

   bazel_dep(name = "ros_central_registry", version = "0.0.1")

Then you can depend on ROS packages like so:

.. code-block:: python

   bazel_dep(name = "rclcpp", version = "...")
