# rosdep (ROS2)

`rosdep` is a command-line tool in ROS2 that installs system dependencies required by ROS2 packages. It ensures you don’t have to manually figure out which OS libraries or external packages are needed. It maps package dependencies declared in **package.xml** files to native system packages across platforms.

---

## Why Use rosdep?
- Automates installation of external system dependencies.
- Keeps ROS2 package building portable across Linux distros and other OSes.
- Ensures consistency when working on multi-machine or CI/CD setups.

---

## Common rosdep Commands (Cheat Sheet)

- `rosdep update`  
  Updates the local database of available dependency rules. Run this first after installing `rosdep`.

- `rosdep check <package>`  
  Checks if all system dependencies of a package are installed.

- `rosdep install --from-paths src --ignore-src -r -y`  
  Installs dependencies for all packages in the `src/` directory, ignoring source-built packages.  
  `-r` = continue even if some deps fail, `-y` = auto-confirm install.

- `rosdep install --reinstall -y -r <package>`  
  Reinstall dependencies for a given package.

- `rosdep keys <package>`  
  Lists all dependency keys that `rosdep` will try to resolve for the package.

- `rosdep resolve <dependency>`  
  Shows what package a dependency maps to on your system.

- `rosdep db`  
  Dumps the entire rosdep rule database to stdout.

---

## Typical Workflow with rosdep (ROS2)
1. Clone a ROS2 workspace (`src/` folder filled with packages).  
2. Run `rosdep update` to sync the rules database.  
3. Run `rosdep install --from-paths src --ignore-src -r -y` to install missing system deps.  
4. Build your workspace with `colcon build`.

---

## Example
- `rosdep install --from-paths src --ignore-src -r -y`  
  → Installs system dependencies for all packages in the `src` folder.  

- `rosdep check nav2_bringup`  
  → Ensures all dependencies for `nav2_bringup` are available before building.

---

## Key Notes
- Must be run with `sudo` when installing system packages.  
- On Debian/Ubuntu, it installs via `apt`; on macOS, via `brew`; on Windows, it may map to `choco`.  
- Works for both ROS1 and ROS2, but is essential for ROS2’s modular ecosystem.

---
```sql
          +------------------+
          | Clone ROS2 repo  |
          +------------------+
                     |
                     v
          +------------------+
          | Run rosdep update|
          +------------------+
                     |
                     v
          +--------------------------------------+
          | rosdep install --from-paths src ...  |
          +--------------------------------------+
                     |
                     v
          +------------------+
          | colcon build     |
          +------------------+
                     |
                     v
          +------------------+
          | Run ROS2 system  |
          +------------------+
```
---
```mermaid
flowchart TD
    A[Clone ROS 2 Package] --> B[Check dependencies with rosdep]
    B -->|rosdep install| C[OS Package Manager (apt/yum/dnf/pip)]
    C --> D[Dependencies Installed]
    D --> E[colcon build workspace]
    E --> F[Run ROS 2 nodes]
```

--- 

| Tool                      | Purpose                                                                | Scope                     | When to Use                                                                |
| ------------------------- | ---------------------------------------------------------------------- | ------------------------- | -------------------------------------------------------------------------- |
| **rosdep**                | Installs system dependencies for ROS packages across distros/platforms | OS package manager + ROS  | Anytime you clone a new ROS package/workspace and need system dependencies |
| **colcon**                | Builds ROS 2 workspaces (compiles packages)                            | Workspace                 | After dependencies are installed with `rosdep`, use it to build packages   |
| **vcs**                   | Manages multiple repositories (clone/update multiple ROS projects)     | Source control            | To fetch/update multiple repos at once (e.g., ros2.repos)                  |
| **rosinstall\_generator** | Generates `.rosinstall` files to fetch packages                        | Source fetching           | If you need to reproduce specific ROS package sets from source             |
| **rosdistro**             | Defines package index and metadata (used by rosdep)                    | Metadata registry         | Not run directly; `rosdep` consults it to map dependencies                 |
| **apt/pip**               | Native OS or Python package manager                                    | System or Python packages | Used under the hood by `rosdep`, but you usually don’t call these directly |

[[ROS2]]
