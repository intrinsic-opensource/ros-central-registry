^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.25.0 (2025-04-25)
-------------------

0.24.1 (2025-04-18)
-------------------
* Fix `__eq__` for Array fields (`#224 <https://github.com/ros2/rosidl_python/issues/224>`_)
* Remove use of ament_target_dependencies (`#222 <https://github.com/ros2/rosidl_python/issues/222>`_)
* Contributors: Michael Carlstrom, Shane Loretz

0.24.0 (2024-11-20)
-------------------
* Revamp how we check for the correct class. (`#218 <https://github.com/ros2/rosidl_python/issues/218>`_)
* Remove python_cmake_module and set hints (`#204 <https://github.com/ros2/rosidl_python/issues/204>`_)
* Contributors: Chris Lalancette

0.23.1 (2024-06-17)
-------------------
* Add rosidl_generator_py to the rosidl_runtime_packages group (`#212 <https://github.com/ros2/rosidl_python/issues/212>`_)
* Contributors: Scott K Logan

0.23.0 (2024-04-26)
-------------------

0.22.0 (2024-04-16)
-------------------
* Revert install of .so files into python path (`#211 <https://github.com/ros2/rosidl_python/issues/211>`_)
  There seems that some regression might have happened after `#195 <https://github.com/ros2/rosidl_python/issues/195>`_.
  When removing those 2 lines, we avoid to install the .so files
  in lib *and* python path.
* Contributors: Matthias Schoepfer

0.21.2 (2024-03-27)
-------------------
* Prototype code for seeing if FindPython3 is usable for rosidl_python (`#140 <https://github.com/ros2/rosidl_python/issues/140>`_)
* Contributors: Shane Loretz

0.21.1 (2024-02-07)
-------------------
* Add in a missing space. (`#203 <https://github.com/ros2/rosidl_python/issues/203>`_)
* Contributors: Chris Lalancette

0.21.0 (2023-12-26)
-------------------
* Install compiled libraries only to 'lib' (`#195 <https://github.com/ros2/rosidl_python/issues/195>`_)
* Contributors: Scott K Logan

0.20.0 (2023-08-21)
-------------------
* Fix: Missing dependency that causes cmake error in downstream (resolves https://github.com/ros2/rosidl_python/issues/198) (`#199 <https://github.com/ros2/rosidl_python/issues/199>`_)
* Contributors: Isaac Saito

0.19.0 (2023-04-28)
-------------------

0.18.0 (2023-04-11)
-------------------
* Hides the assertions that checks the data types of the message fields. (`#194 <https://github.com/ros2/rosidl_python/issues/194>`_)
* Contributors: Eloy Briceno

0.17.0 (2023-02-13)
-------------------
* Service introspection (`#178 <https://github.com/ros2/rosidl_python/issues/178>`_)
* [rolling] Update maintainers - 2022-11-07 (`#189 <https://github.com/ros2/rosidl_python/issues/189>`_)
* Contributors: Audrow Nash, Brian

0.16.1 (2022-11-02)
-------------------
* Remove stray numpy import (`#185 <https://github.com/ros2/rosidl_python/issues/185>`_)
* Contributors: Ben Wolsieffer

0.16.0 (2022-09-13)
-------------------
* :man_farmer: Fix NaN values bound numpy windows version (`#182 <https://github.com/ros2/rosidl_python/issues/182>`_)
* Allow NaN values to pass floating point bounds check. (`#167 <https://github.com/ros2/rosidl_python/issues/167>`_)
* Replace rosidl_cmake imports with rosidl_pycommon (`#177 <https://github.com/ros2/rosidl_python/issues/177>`_)
* Change decode error mode to replace (`#176 <https://github.com/ros2/rosidl_python/issues/176>`_)
* Merge pull request `#173 <https://github.com/ros2/rosidl_python/issues/173>`_ from ros2/quarkytale/fix_import_order
* fix flake
* sorting after conversion
* Revert "Use modern cmake targets to avoid absolute paths to appear in binary archives (`#160 <https://github.com/ros2/rosidl_python/issues/160>`_)" (`#166 <https://github.com/ros2/rosidl_python/issues/166>`_)
* Use modern cmake targets to avoid absolute paths to appear in binary archives (`#160 <https://github.com/ros2/rosidl_python/issues/160>`_)
* michel as author
* adding maintainer
* Contributors: Cristóbal Arroyo, Dharini Dutia, Ivan Santiago Paunovic, Jacob Perron, Tomoya Fujita, quarkytale, Øystein Sture

0.15.0 (2022-05-04)
-------------------

0.14.2 (2022-03-01)
-------------------
* Removes erroneous unmatched closing parenthesis (`#125 <https://github.com/ros2/rosidl_python/issues/125>`_)
* require Python 3.6 as we use format strings in various places (`#152 <https://github.com/ros2/rosidl_python/issues/152>`_)
* Contributors: Charles Cross, William Woodall

0.14.1 (2022-01-13)
-------------------
* Fix rosidl_generator_py assuming incorect library names (`#149 <https://github.com/ros2/rosidl_python/issues/149>`_)
* Fix for msg file containing a property field that is not at the end (`#151 <https://github.com/ros2/rosidl_python/issues/151>`_)
* Update package maintainers (`#147 <https://github.com/ros2/rosidl_python/issues/147>`_)
* Contributors: Chen Lihui, Michel Hidalgo, Shane Loretz

0.14.0 (2021-08-11)
-------------------
* Use rosidl_get_typesupport_target() (`#139 <https://github.com/ros2/rosidl_python/issues/139>`_)
* Contributors: Shane Loretz

0.13.0 (2021-07-16)
-------------------
* Support available typesupport specification in CLI extension (`#133 <https://github.com/ros2/rosidl_python/issues/133>`_)
* Use python_d for test_cli_extension in Debug mode (`#136 <https://github.com/ros2/rosidl_python/issues/136>`_)
* Add missing float/double bounds check (`#128 <https://github.com/ros2/rosidl_python/issues/128>`_)
* Contributors: Michel Hidalgo, Seulbae Kim, Shane Loretz

0.12.0 (2021-06-11)
-------------------
* Added optimization for copying arrays using buffer protocol (`#129 <https://github.com/ros2/rosidl_python/issues/129>`_)
* Add smoke test for CLI extension (`#132 <https://github.com/ros2/rosidl_python/issues/132>`_)
* Install generated Python interfaces in a Python package (`#131 <https://github.com/ros2/rosidl_python/issues/131>`_)
* Contributors: Michel Hidalgo, ksuszka

0.11.0 (2021-03-25)
-------------------
* Remove dependency from rosidl_typesupport_connext_c (`#127 <https://github.com/ros2/rosidl_python/issues/127>`_)
* Contributors: Andrea Sorbini

0.10.0 (2021-03-19)
-------------------
* Expose Python code generation via rosidl generate CLI (`#123 <https://github.com/ros2/rosidl_python/issues/123>`_)
* remove maintainer (`#126 <https://github.com/ros2/rosidl_python/issues/126>`_)
* Contributors: Claire Wang, Michel Hidalgo

0.9.4 (2020-12-08)
------------------
* Update maintainers (`#119 <https://github.com/ros2/rosidl_python/issues/119>`_)
* Fix too early decref of WString when converting from Python to C (`#117 <https://github.com/ros2/rosidl_python/issues/117>`_)
* Add pytest.ini so tests succeed locally. (`#116 <https://github.com/ros2/rosidl_python/issues/116>`_)
* Contributors: Chris Lalancette, Claire Wang, Dirk Thomas

0.9.3 (2020-05-19)
------------------
* Add test_depend on rpyutils (`#115 <https://github.com/ros2/rosidl_python/issues/115>`_)
* Contributors: Jacob Perron

0.9.2 (2020-05-18)
------------------
* Explicitly add DLL directories for Windows before importing (`#113 <https://github.com/ros2/rosidl_python/issues/113>`_)
* Force extension points to be registered in order (`#112 <https://github.com/ros2/rosidl_python/issues/112>`_)
* Contributors: Ivan Santiago Paunovic, Jacob Perron

0.9.1 (2020-04-29)
------------------
* Add test dependency on rosidl_typesupport_c_packages (`#110 <https://github.com/ros2/rosidl_python/issues/110>`_)
* Contributors: Jacob Perron

0.9.0 (2020-04-25)
------------------
* Ensure the Python support target links against the C generator target (`#108 <https://github.com/ros2/rosidl_python/issues/108>`_)
* Skip inoperable typesupport implementations (`#107 <https://github.com/ros2/rosidl_python/issues/107>`_)
* Update includes to use non-entry point headers from detail subdirectory (`#105 <https://github.com/ros2/rosidl_python/issues/105>`_)
* Rename rosidl_generator_c namespace to rosidl_runtime_c (`#103 <https://github.com/ros2/rosidl_python/issues/103>`_)
* Remove dependency on rmw_implementation (`#102 <https://github.com/ros2/rosidl_python/issues/102>`_)
* Added rosidl_runtime c and cpp depencencies (`#100 <https://github.com/ros2/rosidl_python/issues/100>`_)
* Move 'noqa: A003' for fields named like a builtin from property to method line (`#101 <https://github.com/ros2/rosidl_python/issues/101>`_)
* Add warnings for reserved Python keywords in interface members, services and actions (`#96 <https://github.com/ros2/rosidl_python/issues/96>`_)
* Code style only: wrap after open parenthesis if not in one line (`#97 <https://github.com/ros2/rosidl_python/issues/97>`_)
* Use f-string (`#98 <https://github.com/ros2/rosidl_python/issues/98>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Samuel Lindgren

0.8.1 (2019-10-23)
------------------
* Enable tests for 'char' type fields (`#91 <https://github.com/ros2/rosidl_python/issues/91>`_)
* Refactor tests (`#89 <https://github.com/ros2/rosidl_python/issues/89>`_)
* Contributors: Jacob Perron

0.8.0 (2019-09-25)
------------------
* Find numpy headers in non-debian paths (`#75 <https://github.com/ros2/rosidl_python/issues/75>`_) (`#75 <https://github.com/ros2/rosidl_python/issues/75>`_)
* Remove non-package from ament_target_dependencies() (`#76 <https://github.com/ros2/rosidl_python/issues/76>`_)
* Avoid multiple includes for nested array functions (`#72 <https://github.com/ros2/rosidl_python/issues/72>`_)
* Remove the padding member from structs which should be empty (`#73 <https://github.com/ros2/rosidl_python/issues/73>`_)
* Ensure the contents of the field are an array. (`#63 <https://github.com/ros2/rosidl_python/issues/63>`_)
* Make the message __repr_\_ for Python look nicer (`#60 <https://github.com/ros2/rosidl_python/issues/60>`_)
  Before this patch, publishing a message with "ros2 topic pub" would print something like:
  ``publishing #5: my_msgs.msg.my_msg(axes=array('f', [0.0]))``
  While that is OK, it is kind of ugly.
  This patch hides the typecode and the "array" so that the output looks like:
  ``publishing #5: my_msgs.msg.my_msg(axes=[0.0])``
* Contributors: Chris Lalancette, Dirk Thomas, Jacob Perron, Rich Mattes, Shane Loretz

0.7.6 (2019-05-30)
------------------

0.7.5 (2019-05-29)
------------------
* Fix PYTHONPATH for test (`#58 <https://github.com/ros2/rosidl_python/issues/58>`_)
* Contributors: Dirk Thomas

0.7.4 (2019-05-20)
------------------
* Encode/decode strings with UTF-8 (`#57 <https://github.com/ros2/rosidl_python/issues/57>`_)
* Contributors: Dirk Thomas

0.7.3 (2019-05-08 17:57)
------------------------
* Add missing numpy test dependency (`#56 <https://github.com/ros2/rosidl_python/issues/56>`_)
* Contributors: Dirk Thomas

0.7.2 (2019-05-08 16:58)
------------------------
* Fix conversion from C to Python in case a sequence has default values (`#55 <https://github.com/ros2/rosidl_python/issues/55>`_)
* Store types as tuple of abstract types (`#33 <https://github.com/ros2/rosidl_python/issues/33>`_)
* Add WString support (`#47 <https://github.com/ros2/rosidl_python/issues/47>`_)
* Use semantic exec_depend key for python3-numpy. (`#48 <https://github.com/ros2/rosidl_python/issues/48>`_)
* Fix boolean constant in Python mapping (`#46 <https://github.com/ros2/rosidl_python/issues/46>`_)
* Simplify code using updated definition API (`#45 <https://github.com/ros2/rosidl_python/issues/45>`_)
* Update code to match refactoring of rosidl definitions (`#44 <https://github.com/ros2/rosidl_python/issues/44>`_)
* Fix quoted strings for new flake8-quote check. (`#42 <https://github.com/ros2/rosidl_python/issues/42>`_)
* use quotes with least escaping for Python string literals (`#43 <https://github.com/ros2/rosidl_python/issues/43>`_)
* Remove obsolete argument mod_prefix (`#41 <https://github.com/ros2/rosidl_python/issues/41>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Mikael Arguedas, Steven! Ragnarök

0.7.1 (2019-04-14 12:48)
------------------------
* Add numpy dependency to package.xml. (`#39 <https://github.com/ros2/rosidl_python/issues/39>`_)
* Contributors: Steven! Ragnarök

0.7.0 (2019-04-14 05:05)
------------------------
* Fix numpy usage for Windows debug builds (`#36 <https://github.com/ros2/rosidl_python/issues/36>`_)
* Fix compiler warning about unused variable in release mode (`#35 <https://github.com/ros2/rosidl_python/issues/35>`_)
* Map Arrays to numpy.ndarray() and Sequences to array.array() (`#35 <https://github.com/ros2/rosidl_python/issues/35>`_)
* Change generators to IDL-based pipeline (`#24 <https://github.com/ros2/rosidl_python/issues/24>`_)
* Ignore import order on generated imports (`#29 <https://github.com/ros2/rosidl_python/issues/29>`_)
* Provide type support for 'goal_status_array' in action type support
* Fix flake8 error (`#27 <https://github.com/ros2/rosidl_python/issues/27>`_)
* Adds Python typesupport for Actions (`#21 <https://github.com/ros2/rosidl_python/issues/21>`_)
* Contributors: Alexis Pojomovsky, Dirk Thomas, Jacob Perron, Shane Loretz

0.6.2 (2019-01-11)
------------------
* Throw on non-ascii characters in string and char message fields (`#26 <https://github.com/ros2/rosidl_python/issues/26>`_)
* Change uncrustify max line length to 0 (`#25 <https://github.com/ros2/rosidl_python/issues/25>`_)
  This is for compatibility with uncrustify v0.68.
* Contributors: Jacob Perron, Michel Hidalgo

0.6.1 (2018-12-06)
------------------
* Replace deprecated collections usage with collections.abc (`#23 <https://github.com/ros2/rosidl_python/issues/23>`_)
* Adding a get_slot_fields_and_types method to python msg classes (`#19 <https://github.com/ros2/rosidl_python/issues/19>`_)
* Contributors: Dirk Thomas, Mike Lautman, Scott K Logan

0.6.0 (2018-11-16)
------------------
* Allow generated IDL files (`#17 <https://github.com/ros2/rosidl_python/issues/17>`_)
* Rename dynamic array to sequence (`#18 <https://github.com/ros2/rosidl_python/issues/18>`_)
* Added support to msg/srv generation from within an action directory (`#15 <https://github.com/ros2/rosidl_python/issues/15>`_)
* Call conversion functions directly (`#10 <https://github.com/ros2/rosidl_python/issues/10>`_)
  See `#9 <https://github.com/ros2/rosidl_python/issues/9>`_ for more details.
* Fix rosidl target name assumptions (`#12 <https://github.com/ros2/rosidl_python/issues/12>`_)
* Contributors: Alexis Pojomovsky, Dirk Thomas, Martins Mozeiko, Shane Loretz, William Woodall

0.5.2 (2018-07-17)
------------------
* Fixes memory leaks for nested fields (`#7 <https://github.com/ros2/rosidl_python/issues/7>`_)
* Prevent flake8-builtins A003 (`#6 <https://github.com/ros2/rosidl_python/issues/6>`_)
* Contributors: Martins Mozeiko, dhood

0.5.1 (2018-06-28)
------------------
* Fix rosdep key for pytest (`#4 <https://github.com/ros2/rosidl_python/issues/4>`_)
* Use pytest instead of nose (`#3 <https://github.com/ros2/rosidl_python/issues/3>`_)
* Contributors: Dirk Thomas

0.5.0 (2018-06-23)
------------------
* Add groups for generator and runtime packages (`#283 <https://github.com/ros2/rosidl_python/issues/283>`_)
* Support default values for string arrays (`#197 <https://github.com/ros2/rosidl_python/issues/197>`_)
* Generate __eq_\_ for Python messages (`#281 <https://github.com/ros2/rosidl_python/issues/281>`_)
* Add linter tests to message generators (`#278 <https://github.com/ros2/rosidl_python/issues/278>`_)
* Generate imports for assert only in debug mode (`#277 <https://github.com/ros2/rosidl_python/issues/277>`_)
* Use CMAKE_CURRENT_BINARY_DIR for arguments json (`#268 <https://github.com/ros2/rosidl_python/issues/268>`_)
* Declare missing dependency (`#263 <https://github.com/ros2/rosidl_python/issues/263>`_)
* Include directories before invoking rosidl_target_interfaces as the directories added in that macro may contain older version of the same files making them take precedence in the include path (`#261 <https://github.com/ros2/rosidl_python/issues/261>`_)
* 0.4.0
* 0.0.3
* 0.0.2
* Contributors: Brian Gerkey, Dirk Thomas, Ernesto Corbellini, Esteve Fernandez, Hunter Allen, JD Yamokoski, Jackie Kay, Karsten Knese, Martins Mozeiko, Mikael Arguedas, William Woodall, dhood
