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

How do I get a package into the registry?
+++++++++++++++++++++++++++++++++++++++++++++

We only support packages that are present in the upstream rosdistro repository. If you'd like to see a package in the registry, please open an issue against the `rosdistro <https://github.com/ros/rosdistro>`__ repository to request its inclusion.

If your package was recently added to rosdistro, then you will need to wait for the next release to bootstrap the module. Once it has been bootstrapped, you can follow the "patching" instructions in the `developer guide <developer_guide.html>`__ to create a local development environment in which to iterate on your module. Once you have a module, you can open a pull request against the RCR. After mt merges, you should be able to use the module in your own projects. When the next patch release for the distribution is

How do I update your documentation?
+++++++++++++++++++++++++++++++++++

You can checkout the RCR repo and run the following command to preview the documentation locally:

.. code-block:: shell 

   bazel run //docs:preview

This will build the documentation and serve it locally on port 8000. You can then open `http://localhost:8000 <http://localhost:8000>`__ in your browser to view the documentation. It will update automatically as you make changes to the documentation source files in the ``docs`` directory of the repository. Once you are satisfied with your changes, you can open a pull request against the RCR.


How do I deploy build products to a robot?
++++++++++++++++++++++++++++++++++++++++++

Right now we don't have a solid answer for this. Since we build everything from source using a hermetic toolchain, we should in principle be able to cross-compile for different architectures.

However, packaging remains an open problem. Currently, we think that a (yet to be written) ``ros_underlay`` rule might be the right answer to this problem. This rule would collect binaries, libraries, and data files from the workspace into a tarball whose file structure resembles a ROS underlay.  One could then write an application-specific rule that takes this tarball and deploys it to a robot. 

.. code-block:: python 

   ros_underlay(
      name = "my_underlay",
      executables = ["@example//:foo_node"],
      libraries = ["@example//:bar_component"],
      data = ["@example//:baz_data"],
   )

If you have any ideas or suggestions about how we might improve this workflow, please reach out to us.