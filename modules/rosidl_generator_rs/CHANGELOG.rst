^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_rs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.7 (2025-05-30)
------------------
* fix: clean up dependencies (`#5 <https://github.com/ros2-rust/rosidl_rust/issues/5>`_)
* Contributors: Esteve Fernandez

0.4.6 (2025-05-24)
------------------
* fix: added missing dependency
* Contributors: Esteve Fernandez

0.4.5 (2025-05-24)
------------------
* clean up changelog. Removed rosidl_runtime_rs as a dependency
* Contributors: Esteve Fernandez

0.4.4 (2025-05-24)
------------------
* set python executable var to custom cmake commands (`#3 <https://github.com/ros2-rust/rosidl_rust/issues/3>`_)
* Contributors: Kimberly N. McGuire

0.4.3 (2025-04-15)
------------------
* Disable rosidl_runtime_rs dependency (`#4 <https://github.com/ros2-rust/rosidl_rust/issues/4>`_)
* Contributors: Esteve Fernandez

0.4.2 (2025-04-11)
------------------
* Update vendored interface packages (`#423 <https://github.com/ros2-rust/rosidl_rust/issues/423>`_)
  * Update rclrs vendor_interfaces.py script
  This updates the vendor_interfaces.py script to also vendor in the
  action_msgs and unique_identifier_msgs packages. The script is modified
  to always use a list of package names rather than hard-coding the
  package names everywhere.
  * Silence certain clippy lints on generated code
  In case a user enforces clippy linting on these generated packages,
  silence expected warnings. This is already the case in rclrs, but should
  be applied directly to the generated packages for the sake of downstream
  users.
  The `clippy::derive_partial_eq_without_eq` lint was already being
  disabled for the packages vendored by rclrs, but is now moved to the
  rosidl_generator_rs template instead. This is necessary since we always
  derive the PartialEq trait, but can't necessary derive Eq, and so don't.
  The `clippy::upper_case_acronyms` is new and was added to account for
  message type names being upper-case acrynyms, like
  unique_identifier_msgs::msg::UUID.
  * Update vendored message packages
  This updates the message packages vendored under the rclrs `vendor`
  module. The vendor_interfaces.py script was used for reproducibility.
  The existing packages – rcl_interfaces, rosgraph_msgs, and
  builtin_interfaces – are only modified in that they now use the
  std::ffi::c_void naming for c_void instead of the std::os::raw::c_void
  alias and disable certain clippy lints.
  The action_msgs and unique_identifier_msgs packages are newly added to
  enable adding action support to rclrs. action_msgs is needed for
  interaction with action goal info and statuses, and has a dependency on
  unique_identifier_msgs.
* Action message support (`#417 <https://github.com/ros2-rust/rosidl_rust/issues/417>`_)
  * Added action template
  * Added action generation
  * Added basic create_action_client function
  * dded action generation
  * checkin
  * Fix missing exported pre_field_serde field
  * Removed extra code
  * Sketch out action server construction and destruction
  This follows generally the same pattern as the service server. It
  required adding a typesupport function to the Action trait and pulling
  in some more rcl_action bindings.
  * Fix action typesupport function
  * Add ActionImpl trait with internal messages and services
  This is incomplete, since the service types aren't yet being generated.
  * Split srv.rs.em into idiomatic and rmw template files
  This results in the exact same file being produced for services,
  except for some whitespace changes. However, it enables actions to
  invoke the respective service template for its generation, similar to
  how the it works for services and their underlying messages.
  * Generate underlying service definitions for actions
  Not tested
  * Add runtime trait to get the UUID from a goal request
  C++ uses duck typing for this, knowing that for any `Action`, the type
  `Action::Impl::SendGoalService::Request` will always have a `goal_id`
  field of type `unique_identifier_msgs::msg::UUID` without having to
  prove this to the compiler. Rust's generics are more strict, requiring
  that this be proven using type bounds.
  The `Request` type is also action-specific as it contains a `goal` field
  containing the `Goal` message type of the action. We therefore cannot
  enforce that all `Request`s are a specific type in `rclrs`.
  This seems most easily represented using associated type bounds on the
  `SendGoalService` associated type within `ActionImpl`. To avoid
  introducing to `rosidl_runtime_rs` a circular dependency on message
  packages like `unique_identifier_msgs`, the `ExtractUuid` trait only
  operates on a byte array rather than a more nicely typed `UUID` message
  type.
  I'll likely revisit this as we introduce more similar bounds on the
  generated types.
  * Integrate RMW message methods into ActionImpl
  Rather than having a bunch of standalone traits implementing various
  message functions like `ExtractUuid` and `SetAccepted`, with the
  trait bounds on each associated type in `ActionImpl`, we'll instead add
  these functions directly to the `ActionImpl` trait. This is simpler on
  both the rosidl_runtime_rs and the rclrs side.
  * Add rosidl_runtime_rs::ActionImpl::create_feedback_message()
  Adds a trait method to create a feedback message given the goal ID and
  the user-facing feedback message type. Depending on how many times we do
  this, it may end up valuable to define a GoalUuid type in
  rosidl_runtime_rs itself. We wouldn't be able to utilize the
  `RCL_ACTION_UUID_SIZE` constant imported from `rcl_action`, but this is
  pretty much guaranteed to be 16 forever.
  Defining this method signature also required inverting the super-trait
  relationship between Action and ActionImpl. Now ActionImpl is the
  sub-trait as this gives it access to all of Action's associated types.
  Action doesn't need to care about anything from ActionImpl (hopefully).
  * Add GetResultService methods to ActionImpl
  * Implement ActionImpl trait methods in generator
  These still don't build without errors, but it's close.
  * Replace set_result_response_status with create_result_response
  rclrs needs to be able to generically construct result responses,
  including the user-defined result field.
  * Implement client-side trait methods for action messages
  This adds methods to ActionImpl for creating and accessing
  action-specific message types. These are needed by the rclrs
  ActionClient to generically read and write RMW messages.
  Due to issues with qualified paths in certain places
  (https://github.com/rust-lang/rust/issues/86935), the generator now
  refers directly to its service and message types rather than going
  through associated types of the various traits. This also makes the
  generated code a little easier to read, with the trait method signatures
  from rosidl_runtime_rs still enforcing type-safety.
  * Format the rosidl_runtime_rs::ActionImpl trait
  * Wrap longs lines in rosidl_generator_rs action.rs
  This at least makes the template easier to read, but also helps with the
  generated code. In general, the generated code could actually fit on one
  line for the function signatures, but it's not a big deal to split it
  across multiple.
  * Use idiomatic message types in Action trait
  This is user-facing and so should use the friendly message types.
  * Cleanup ActionImpl using type aliases
  * Formatting
  * Switch from std::os::raw::c_void to std::ffi::c_void
  While these are aliases of each other, we might as well use the more
  appropriate std::ffi version, as requested by reviewers.
  * Clean up rosidl_generator_rs's cmake files
  Some of the variables are present but no longer used. Others were not
  updated with the action changes.
  * Add a short doc page on the message generation pipeline
  This should help newcomers orient themselves around the rosidl\_*_rs
  packages.
  ---------
  Co-authored-by: Esteve Fernandez <esteve@apache.org>
  Co-authored-by: Michael X. Grey <grey@openrobotics.org>
* Declare rust_packages only when installing Rust IDL bindings (`#380 <https://github.com/ros2-rust/rosidl_rust/issues/380>`_)
* Allow ros2_rust to be built within a distro workspace
* Add wchar support (`#349 <https://github.com/ros2-rust/rosidl_rust/issues/349>`_)
  * Add wchar support and .idl example
  * Undo automatic IDE formatting noise
  * Added back unused imports to see if this fixes the build
  * More attempts to fix the weird build failure
  * Removed the linter tests for auto-generated message source files in `rclrs_example_msgs`. Re-applied some changes removed when root causing.
  ---------
  Co-authored-by: Sam Privett <sam@privett.dev>

0.4.1 (2023-11-28)
------------------
* Version 0.4.1 (`#353 <https://github.com/ros2-rust/rosidl_rust/issues/353>`_)

0.4.0 (2023-11-07)
------------------
* Version 0.4.0 (`#346 <https://github.com/ros2-rust/rosidl_rust/issues/346>`_)
* Revert "Version 0.4.0 (`#343 <https://github.com/ros2-rust/rosidl_rust/issues/343>`_)" (`#344 <https://github.com/ros2-rust/rosidl_rust/issues/344>`_)
  This reverts commit a64e397990319db39caf79ef7863b21fb2c828ea.
* Version 0.4.0 (`#343 <https://github.com/ros2-rust/rosidl_rust/issues/343>`_)
* add serde big array support (fixed `#327 <https://github.com/ros2-rust/rosidl_rust/issues/327>`_) (`#328 <https://github.com/ros2-rust/rosidl_rust/issues/328>`_)
  * add serde big array support
* Swapped usage of rosidl_cmake over to the new rosidl_pycommon. (`#297 <https://github.com/ros2-rust/rosidl_rust/issues/297>`_)
  * Swapped usage of rosidl_cmake over to the new rosidl_pycommon.
  As of [rosidl 3.3.0](https://github.com/ros2/rosidl/commit/9348ce9b466335590dc334aab01f4f0dd270713b), the rosidl_cmake Python module was moved to a new rosidl_pycommon package and the Python module in rosidl_cmake was deprecated.
  * Support builds from older ROS 2 distros.
  * Fixed build for rolling
  * Added `test_depend` conditional inclusion as well.
  * Swap to a more elegant check
  * PR Feedback
  ---------
  Co-authored-by: Sam Privett <sam@privett.dev>
* Remove libc dependencies (`#284 <https://github.com/ros2-rust/rosidl_rust/issues/284>`_)

0.3.1 (2023-08-22)
------------------
* Version 0.3.1 (`#285 <https://github.com/ros2-rust/rosidl_rust/issues/285>`_)
* Add TYPE_NAME constant to messages and make error fields public (`#277 <https://github.com/ros2-rust/rosidl_rust/issues/277>`_)
* Bump package versions to 0.3 (`#274 <https://github.com/ros2-rust/rosidl_rust/issues/274>`_)
* Add support for constants to message generation (`#269 <https://github.com/ros2-rust/rosidl_rust/issues/269>`_)
  This will produce:
  ```
  impl VariousTypes {
  /// binary, hexadecimal and octal constants are also possible
  pub const TWO_PLUS_TWO: i8 = 5;
  /// Only unbounded strings are possible
  pub const PASSWORD: &'static str = "hunter2";
  /// As determined by Edward J. Goodwin
  pub const PI: f32 = 3.0;
  }
  ```
* Small bugfix for sequences of WStrings (`#240 <https://github.com/ros2-rust/rosidl_rust/issues/240>`_)
  Message packages containing unbounded sequences of WStrings, like test_msgs, would not compile because of this.
* Fix path handling in rosidl_generator_rs on Windows (`#228 <https://github.com/ros2-rust/rosidl_rust/issues/228>`_)
  Paths on Windows can contain colons. With rsplit, the drive letter was
  grouped with the package name.
* Added support for clients and services (`#146 <https://github.com/ros2-rust/rosidl_rust/issues/146>`_)
  * Added support for clients and services
* feat: obtain interface version from cmake variable (`#191 <https://github.com/ros2-rust/rosidl_rust/issues/191>`_)
  * feat: obtain interface version from cmake variable
  * refactor: append package version into generator arguments file
* Add build.rs to messages to automatically find the message libraries (`#140 <https://github.com/ros2-rust/rosidl_rust/issues/140>`_)
* Generate Cargo.toml of message crate with an EmPy template, not CMake (`#138 <https://github.com/ros2-rust/rosidl_rust/issues/138>`_)
  * Generate Cargo.toml of message crate with an EmPy template, not CMake
  * Add comment
* Add serde support to messages (`#131 <https://github.com/ros2-rust/rosidl_rust/issues/131>`_)

0.2.0 (2022-04-17)
------------------
* Bump every package to version 0.2 (`#100 <https://github.com/ros2-rust/rosidl_rust/issues/100>`_)
* Enable Clippy in CI (`#83 <https://github.com/ros2-rust/rosidl_rust/issues/83>`_)
* Message generation refactoring (`#80 <https://github.com/ros2-rust/rosidl_rust/issues/80>`_)
  Previously, only messages consisting of basic types and strings were supported. Now, all message types will work, including those that have fields of nested types, bounded types, or arrays.
  Changes:
  - The "rsext" library is deleted
  - Unused messages in "rosidl_generator_rs" are deleted
  - There is a new package, "rosidl_runtime_rs", see below
  - The RMW-compatible messages from C, which do not require an extra conversion step, are exposed in addition to the "idiomatic" messages
  - Publisher and subscription are changed to work with both idiomatic and rmw types, through the unifying `Message` trait
  On `rosidl_runtime_rs`: This package is the successor of `rclrs_msg_utilities` package, but doesn't have much in common with it anymore.
  It provides common types and functionality for messages. The `String` and `Sequence` types and their variants in that package essentially wrap C types from the `rosidl_runtime_c` package and C messages generated by the "rosidl_generator_c" package.
  A number of functions and traits are implemented on these types, so that they feel as ergonomic as possible, for instance, a `seq!` macro for creating a sequence. There is also some documentation and doctests.
  The memory for the (non-pretty) message types is managed by the C allocator.
  Not yet implemented:
  - long double
  - constants
  - Services/clients
  - @verbatim comments
  - ndarray for sequences/arrays of numeric types
  - implementing `Eq`, `Ord` and `Hash` when a message contains no floats
* Use the ament_cargo build type (`#73 <https://github.com/ros2-rust/rosidl_rust/issues/73>`_)
  * Use the ament_cargo build type
  The rclrs_crate_config_generator is superseded by colcon-ros-cargo.
  The ament_cmake_export_crates mechanism is subsumed by creating entries in the ament index directly in the rosidl_generator_rs and cargo-ament-build.
  * Install colcon-cargo and colcon-ros-cargo
  * Force running pip3 as root
  * Install cargo-ament-build
  * Removed no longer needed dependencies
  * Disable Rolling job
  * Update README
  * Update rust.yml
  * Update README.md
  Co-authored-by: Esteve Fernandez <esteve@apache.org>
* Build system refactor (`#64 <https://github.com/ros2-rust/rosidl_rust/issues/64>`_)
  * Experimental change to build system.
  Allows IDE to parse dependencies.
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Remove commented code
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Refactoring to workspace layout. Does not compile.
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Revert change to workspace, general CMake tweaks
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Initial re-make of build system
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Fixing warnings within rosidl_generator
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Make sure cargo builds within the correct directory
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Add in checks for ROS 2 version to change
  the compilation syntax
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Properly query environment variable
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Only bind rcl, rmw, and rcutils
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Re-write to move most of `rclrs_common` to `rclrs`
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Updating/fixing package XML to comply with
  format 3 schema
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Missed a schema update
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Missed another schema...
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
  * Remove manual crate paths in toml files
  Distro A, OPSEC `#4584 <https://github.com/ros2-rust/rosidl_rust/issues/4584>`_. You may have additional rights; please see https://rosmilitary.org/faq/?category=ros-2-license
* Fix array type generation. And append an '_' to field names that an rust keywords. (`#30 <https://github.com/ros2-rust/rosidl_rust/issues/30>`_)
* Build on Dashing+ (`#24 <https://github.com/ros2-rust/rosidl_rust/issues/24>`_)
  * fix warnings
  * update README for Ubuntu 18.04
  * Build on Dashing
  * Build on Eloquent
  * Build on Foxy
  * clean in IDL generator
  * Use foxy in pipeline
  Co-authored-by: deb0ch <tom@blackfoot.io>
  Co-authored-by: deb0ch <thomas.de.beauchene@gmail.com>
* Crystal and more (`#3 <https://github.com/ros2-rust/rosidl_rust/issues/3>`_)
  * nested messages working
  * fix array support
  * add rcl_sys
  * add author & fix compilation order
  * readme
  * format
  * fix clippy warnings
  * delete patch
  * remove leftover build.rs
  * fix authors
  * add qos support
  * add spin & change handle handling
  * clippy
  * edit readme
  * Update README.md
  * fix message generation issue
  * remove messages
  * fix fixed size nested array issue
  * delete unused files
  * reset authors
  * remove rcl_sys
  * remove remaining authors & revert readme
  * fix quickstart
  * fix fixed size array warning
  * add rosidl_defaults to repos
  * fix warnings with array generation
  * register the 'rosidl_generator_rs'
  * revert message generation to its initial state
  * add rcl build dependency to rclrs
  * move spin and spin_once from Node to rclrs
  * move publisher sleep at the end of the loop
  * re-add msg to rosidl_generator_rs
  * add TODO for publisher and subscription lifetime
* Initial implementation
* Contributors: Daisuke Nishimatsu, Esteve Fernandez, Fawdlstty, Grey, Gérald Lelong, Michael X. Grey, Nathan Wiebe Neufeldt, Nikolai Morin, Sam Privett, Tatsuro Sakaguchi, jhdcs, nnarain
