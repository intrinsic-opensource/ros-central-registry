^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcl_logging_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.3.0 (2025-04-25)
------------------

3.2.2 (2024-10-03)
------------------

3.2.1 (2024-07-29)
------------------

3.2.0 (2024-04-26)
------------------

3.1.0 (2024-03-28)
------------------
* Check allocator validity in some rcl_logging functions (`#116 <https://github.com/ros2/rcl_logging/issues/116>`_)
  If the allocator is zero-initialized, it may cause a segfault when it is
  used later in the functions.
* Use (void) in declaration of param-less function (`#114 <https://github.com/ros2/rcl_logging/issues/114>`_)
* Contributors: Christophe Bedard, Scott K Logan

3.0.0 (2024-01-24)
------------------
* add file_name_prefix parameter to external log configuration. (`#109 <https://github.com/ros2/rcl_logging/issues/109>`_)
* Contributors: Tomoya Fujita

2.7.1 (2023-12-26)
------------------
* Migrate to std::filesystem (`#104 <https://github.com/ros2/rcl_logging/issues/104>`_)
* Contributors: Kenta Yonekura

2.7.0 (2023-09-07)
------------------
* Remove the last uses of ament_target_dependencies in this repo. (`#102 <https://github.com/ros2/rcl_logging/issues/102>`_)
* Contributors: Chris Lalancette

2.6.0 (2023-04-27)
------------------

2.5.1 (2023-04-11)
------------------

2.5.0 (2023-02-13)
------------------
* Update rcl_logging to C++17. (`#98 <https://github.com/ros2/rcl_logging/issues/98>`_)
* Contributors: Chris Lalancette

2.4.3 (2022-11-18)
------------------
* Updated maintainers - 2022-11-07 (`#96 <https://github.com/ros2/rcl_logging/issues/96>`_)
* Contributors: Audrow Nash

2.4.2 (2022-11-02)
------------------

2.4.1 (2022-09-13)
------------------

2.4.0 (2022-04-29)
------------------

2.3.0 (2022-03-01)
------------------
* Install includes to include/${PROJECT_NAME} (`#85 <https://github.com/ros2/rcl_logging/issues/85>`_)
* Contributors: Shane Loretz

2.2.1 (2022-01-14)
------------------
* Fix include order for cpplint (`#84 <https://github.com/ros2/rcl_logging/issues/84>`_)
  Relates to https://github.com/ament/ament_lint/pull/324
* Update maintainers to Chris Lalancette (`#83 <https://github.com/ros2/rcl_logging/issues/83>`_)
* Contributors: Audrow Nash, Jacob Perron

2.2.0 (2021-11-18)
------------------
* Fix renamed `rcpputils` header (`#81 <https://github.com/ros2/rcl_logging/issues/81>`_)
* Contributors: Abrar Rahman Protyasha

2.1.3 (2021-09-16)
------------------
* Add Doxyfile to rcl_logging_interface package (`#80 <https://github.com/ros2/rcl_logging/issues/80>`_)
* Update includes after rcutils/get_env.h deprecation (`#75 <https://github.com/ros2/rcl_logging/issues/75>`_)
* Contributors: Christophe Bedard, Michel Hidalgo

2.1.2 (2021-04-06)
------------------

2.1.1 (2021-01-25)
------------------

2.1.0 (2020-12-08)
------------------
* Update QD to QL 1 (`#66 <https://github.com/ros2/rcl_logging/issues/66>`_)
* Use rcutils_expand_user in rcl_logging_get_logging_directory (`#59 <https://github.com/ros2/rcl_logging/issues/59>`_)
* Allow configuring logging directory through environment variables (`#53 <https://github.com/ros2/rcl_logging/issues/53>`_)
* Update the maintainers. (`#55 <https://github.com/ros2/rcl_logging/issues/55>`_)
* Contributors: Chris Lalancette, Christophe Bedard, Stephen Brawner

2.0.1 (2020-07-21)
------------------

2.0.0 (2020-06-18)
------------------
* Add new package with rcl logging interface (`#41 <https://github.com/ros2/rcl_logging/issues/41>`_)
* Contributors: Chris Lalancette
