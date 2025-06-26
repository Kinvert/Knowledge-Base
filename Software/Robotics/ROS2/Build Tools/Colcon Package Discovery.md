# Colcon Package Discovery in ROS 2

## Overview

**colcon** is the standard build tool for ROS 2 workspaces. Its package discovery mechanism is designed to flexibly and recursively locate all valid ROS 2 packages within your workspace's source directory.

---

## How [[Colcon]] Discovers Packages

- **Recursive Directory Traversal:**  
  When you run `colcon build` from the root of your workspace (e.g., `~/ros2_ws`), colcon recursively scans the `src` subdirectory by default, walking through all nested folders.

- **Package Identification:**  
  A directory is recognized as a package if it contains a `package.xml` file.  
    - For CMake-based packages, a `CMakeLists.txt` is typically also required.
    - For Python-based packages, a `setup.py` or `setup.cfg` is expected.
  Any directory (even deeply nested) with a `package.xml` is treated as a ROS 2 package.

- **Nested Packages:**  
  Packages can be nested within other directories inside `src` (e.g., `src/SDK/ros2`). As long as the directory contains a valid `package.xml`, it will be discovered and built.  
  - If a parent directory (like `SDK`) has its own `CMakeLists.txt` or `package.xml`, colcon may attempt to treat it as a package as well, which can lead to errors if not intended.

- **Ignoring Directories:**  
  To prevent colcon from scanning certain directories, place an empty file named `COLCON_IGNORE` in those directories.

- **Custom Source Paths:**  
  While `src` is the convention, colcon can scan any directory. Use the `--base-paths` or `--paths` options to specify alternative locations.

---

## Summary Table

| Step                         | What colcon Does                                                                 |
|------------------------------|----------------------------------------------------------------------------------|
| Start in workspace root      | Looks for a `src` folder by convention, but scans current directory recursively  |
| Recursively scan directories | Searches all subdirectories for valid ROS 2 packages                             |
| Identify packages            | Looks for `package.xml` (and build files like `CMakeLists.txt` or `setup.py`)    |
| Handle nested packages       | Discovers packages at any depth, as long as they have a `package.xml`            |
| Ignore unwanted directories  | Skips directories containing a `COLCON_IGNORE` file                              |
| Custom source locations      | Can use `--base-paths` or `--paths` to specify alternative locations             |

---

## References

- [ROS 2 Foxy colcon tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS 2 Humble colcon tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [colcon Discovery Arguments Documentation](https://colcon.readthedocs.io/en/released/reference/discovery-arguments.html)
- [ROS Industrial Training](https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS2-Basics.html)

---

## See Also
- [[package.xml]]
- [[setup.py]]
