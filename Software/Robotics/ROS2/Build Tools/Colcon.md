# 🟣 colcon build

**colcon build** is the standard build tool for **ROS2** and other modern CMake-based multi-package workspaces. It’s designed to build, test, and install multiple packages in a workspace, handling complex dependency trees efficiently.

---

## 🧠 Summary

- colcon stands for **COllective CONstruction**.
- It replaces `catkin_make`, `catkin_tools`, and `ament_tools`.
- Supports **parallel builds**, **isolated builds**, and **build summaries**.
- Works well with both ROS 2 and non-ROS packages in a single workspace.

---

## ⚙️ Typical Usage

Inside your workspace root (e.g. `~/ros2_ws`):

- `colcon build` – Build all packages
- `colcon build --packages-select <pkg_name>` – Build only specified package(s)
- `colcon build --symlink-install` – Use symlinks for install space (useful for development)
- `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` – Pass custom CMake args

---

## 🗂️ Workspace Layout Example

```
ros2_ws/
├── src/
│ ├── my_robot/
│ └── simple_camera/
├── build/
├── install/
└── log/
```

- `src/` contains your source code.
- `build/` holds intermediate build files.
- `install/` contains final executables, libraries, and headers (after build).

---

## 🚀 How It Works

1. **Source Discovery:** colcon scans the `src/` directory for packages (CMake, Python, etc).
2. **Dependency Resolution:** It figures out the order to build packages based on dependencies.
3. **Build:** Invokes CMake (or other build systems) in the right order.
4. **Install:** Packages are installed into the `install/` directory (or wherever you specify).

When you do:

`colcon build`

It builds each package and installs it to `install/`.

---

## 🤔 Why `ros2 run` Sometimes Fails to Find Packages

`ros2 run` and other ROS tools rely on environment variables like:

- `AMENT_PREFIX_PATH`
- `PATH`
- `PYTHONPATH`

These are set when you **source the local setup scripts**:

`source install/setup.bash`

If you forget to source this, `ros2 run` won’t know where your packages are installed.

---

## 🤔 Why You Can Sometimes Run Directly Like `./install/simple_camera/bin/simple_publisher`

When colcon builds and installs your packages:

- Executables are placed under `install/<pkg>/bin/`.
- If you know the path, you can run the binary directly.

However, this skips environment setup, so things like parameters, ROS topic names, and plugins might not work as expected.

---

## ⚠️ Common Problems

| Issue | Cause | Solution |
|-------|-------|----------|
| `ros2 run` can't find your package | You didn’t source `install/setup.bash` | Run `source install/setup.bash` |
| Build fails on missing dependencies | System dependency missing | Install via apt or rosdep |
| Changes aren’t reflected after build | `--symlink-install` not used, or forgot to rebuild | Use `--symlink-install`, rebuild package |
| Python package changes not showing | Python cache issue, or forgot to rebuild | Remove `build/`, `install/`, `log/` and rebuild |
| Stale build artifacts causing errors | Leftover files in `build/` or `install/` | Clean build: `rm -rf build/ install/ log/` then `colcon build` |
| CMake arg not applied | Forgot `--cmake-args` or syntax error | Double check your command, e.g., `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` |

---

## 🏆 Strengths

- Handles complex workspaces cleanly.
- Supports mixed-language packages.
- Provides detailed build summaries.
- Integrates with testing (`colcon test`).

---

## ⚠️ Weaknesses

- More verbose than older ROS 1 tools.
- Can be slow for large workspaces without tuning.
- Beginners often forget to source `install/setup.bash`.

---

## 🔗 Related Notes

- [[ROS2]]
- [[ROS2 Node]]
- [[ROS2 Topics]]
- [[rclpy]]
- [[sensor_msgs]]

---

## 🌐 External References

- [colcon official docs](https://colcon.readthedocs.io)
- [ROS2 Build System Overview](https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html)

---
