# Colcon Cheatsheet

[[Colcon]] is the primary build tool for ROS 2 (and other packages). It extends CMake/ament functionality for multi-package builds.

---

## Common Commands

`colcon build`  
Build all packages in the workspace.

`colcon build --packages-select <pkg>`  
Build only the specified package.

`colcon build --packages-up-to <pkg>`  
Build `<pkg>` and all dependencies up to it.

`colcon build --packages-above <pkg>`  
Build packages that depend on `<pkg>`.

`colcon build --packages-skip <pkg>`  
Skip building a specified package.

`colcon build --symlink-install`  
Symlink install directories (good for development).

`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`  
Pass arguments directly to CMake.

`colcon build --parallel-workers <n>`  
Set number of parallel build workers.

---

## Running Tests

`colcon test`  
Run all tests in the workspace.

`colcon test --packages-select <pkg>`  
Run tests for a specific package.

`colcon test-result`  
Show aggregated test results.

`colcon test-result --verbose`  
Show detailed test results.

---

## Cleaning

`colcon build --cmake-clean-cache`  
Clean CMake cache for packages before rebuilding.

`rm -rf build/ install/ log/`  
Manual clean of workspace (reset state).

---

## Running

`ros2 run <pkg> <executable>`  
Run an executable from a package (via ROS 2).

---

## Workspace Management

`colcon list`  
List all packages in the workspace.

`colcon list --packages-select <pkg>`  
List specific package info.

`colcon graph`  
Show the dependency graph of the workspace.

---

## Comparing with Other Tools

| Tool      | Purpose                          | Notes                                                                 |
|-----------|----------------------------------|-----------------------------------------------------------------------|
| **colcon**| Build tool for ROS 2 workspaces  | Handles multiple packages, dependencies, and testing in one workspace |
| **catkin_make** | ROS 1 build tool              | Easier but less powerful; single CMake build context                  |
| **catkin_tools** | Improved ROS 1 build system | Similar UX to colcon, but not used in ROS 2                          |
| **CMake** | General build system              | Used under the hood by colcon                                         |
| **make**  | Legacy build system               | Much lower-level, no ROS-specific logic                               |
