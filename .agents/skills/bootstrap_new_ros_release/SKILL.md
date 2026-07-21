---
name: bootstrap_new_ros_release
description: >-
  Imports a new ROS release into the RCR from https://github.com/ros/rosdistro. This creates any new modules that are needed, including a special "0.0.0" module version used to make importing dependnencies easier for usets. After adding new module versions for each ROS package in the release, it copies over the patches from the previous release to create new module versions ending in .rcr.0. After this, the agent calls tooling to vendor the entire workspace, and fix any patching errors that arise. The final patch set is then rolled up into modules ending with .rcr.1.
---

# Bootstrap new ROS release

This skill provides the exact procedure for importing a new ROS release into the RCR. To do this, it calls several internal tools.

## Prerequisites

- [ ] Ensure you are at the root of the `ros-central-registry` repository.
- [ ] Ensure the user has provided the new release information: a ROS distribution name (eg. `lyrical`), release date (eg `2026-06-23`), ubuntu distribution name (eg. `resolute`) and ubuntu architecture (eg. `arm64` or `amd64`).

## Procedure

### 1. Prepare the new release

Copy this checklist and track progress:
- [ ] Step 1: Call the release_bootstrap code to generate the initial set of modules
```bash
bazel run //tools:release_bootstrap -- $PWD \
    --ros-release-distro <ros-release-name> \
    --ros-release-date <ros-release-date> \
    --ubuntu-distro <ubuntu-distribution-name> \
    --ubuntu-architecture <ubuntu-architecture>
```
- [ ] Step 2: Call the `release_migrate` tool to copy the patch set from the most recent release to the new release.
```bash
bazel run //tools:release_migrate -- $PWD \
    --release <ros-release-name>.<ros-release-date> \
    --overwrite
```
- [ ] Step 3: Call the `workspace_setup` tool to set up two workspaces for the release -- one for the vanilla checkout and the other for the patched checkout.
```bash
bazel run //tools:workspace_setup -- $PWD \
    --release <ros-release-name>.<ros-release-date> \
    --overwrite
```

### 2. Vendor the workspaces

In the last step of the previous section the tool printed out two new directories under the workspace folder. These are the vanilla checkout and the patched checkout. Vendor both of these directories by calling `bazel vendor` from inside of each directory.

Copy this checklist and track progress:
- [ ] Step 1: Vendor the vanilla check out
```bash
pushd workspace/<ros-release-name>.<ros-release-date>
bazel vendor
popd
```
- [ ] Step 2: Vendor the patched check out
```bash
pushd workspace/<ros-release-name>.<ros-release-date>.rcr.1
bazel vendor
popd
```

The first step should work without error. You might encounter patching errors in the second step. If you do, simply disable the broken patches in the source.json 


Attempt to redsolve them by looking at the patches for the mopdule that failed, and fix the code directly in the patched checkout. 

### 3. Initialize Files
You should find that two new workspaces are created

### 4. Update Metadata
Add the new version to `modules/{module_name}/metadata.json`.