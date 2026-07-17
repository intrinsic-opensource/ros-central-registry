#!/bin/bash
# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
set -e

# Delete all existing modules and workspaces.
rm -rf modules \
    workspace/rolling.2026-01-21 \
    workspace/rolling.2026-01-21.rcr.1

# Bootstrap a release using the rolling distro and noble ubuntu.
# This will add modules to ./modules with no suffix.
bazel run //tools:release_bootstrap -- $PWD \
    --ros-release-distro rolling \
    --ros-release-date 2026-01-21 \
    --ubuntu-distro noble \
    --ubuntu-architecture amd64

# Migrate all patches from the previous release to the new release.
# This will add modules to modules with a .rcr.0 suffix.
bazel run //tools:release_migrate -- $PWD \
    --release rolling.2026-01-21 \
    --overwrite

# Create the new workspace to add some additional patches.
# This will add two things to ./workspace
# - a "rolling.2026-01-21" folder, which we call the REF release.
# - a "rolling.2026-01-21.rcr.1" folder, which we call the NEW release.
bazel run //tools:workspace_setup -- $PWD \
    --release rolling.2026-01-21 \
    --overwrite

# Vendor all source packages in the two workspaces.
pushd workspace/rolling.2026-01-21
bazel vendor
popd
pushd workspace/rolling.2026-01-21.rcr.1
bazel vendor
popd

# Strip all existing starlark files and autogenerate:
bazel run //tools:helper_strip_starlark_files -- $PWD \
    --workspace rolling.2026-01-21.rcr.1
bazel run //tools:helper_interface_autogen -- $PWD \
    --workspace rolling.2026-01-21.rcr.1

# # Calculate the diff between the two NEW and the REF release, and transform
# # these differences into new modules with elevated patch suffixes.
bazel run //tools:workspace_patch -- $PWD \
    --workspace rolling.2026-01-21.rcr.1 \
    --dry-run \
    --overwrite

# # Once a patch has been applied it can be reverted from the 'modules'.
# bazel run //tools:workspace_revert -- $PWD \
#     --workspace rolling.2026-01-21.rcr.1


bazel run //tools:release_bootstrap -- $PWD \
    --ros-release-distro rolling \
    --ros-release-date 2026-04-15 \
    --ubuntu-distro resolute \
    --ubuntu-architecture amd64