FAQs
====

Why not use a module extension?
+++++++++++++++++++++++++++++++
If you've been working with Bazel for a while, you might have come across the ``pip`` or ``crates`` extensions in ``rules_python`` or ``rules_rust`` respectively. You might wonder why we don't support a usage pattern that offers something like the following for convenience:

.. code-block:: python
   :caption: MODULE.bazel

   bazel_dep(name = "rosdistro", version = "rolling.2026-02-19.bcr.1")

   ...

   load("@rosdistro//bazel:ros.bzl", "ros")
   ros.load("sensor_msgs")
   ros.load("rclcpp")
   use_repo(ros, "ros")


Unfortunately, this is not possible because of the order in which Bazel phases its execution. In the first resolution phase the ``MODULE.bazel`` file is parsed and dependencies are resolved. In teh second extension phase the module extensions are loaded and executed. In summary, we wouldn't be able to distribute ROS packages as Bazel modules, and therefore we'd lose the dependency and versioning benefits. In stead, we'd have to resort to the older ``http_archive`` or ``git_repository`` rules.