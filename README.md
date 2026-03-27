# ackermann_trajectory_control

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
</p>

**TODO: Repository tagline/description**

TODO: High-level repository introduction paragraph

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>🧑‍💻 <a href="%E2%80%8D-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>

> [!IMPORTANT]
> This repository is part of [🚗 ***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Stack*.


<!-- <img src="TODO: teaser image/gif" width=800> -->


## 🚀 Quick Start

> [!NOTE]
> Example only: replace this section with repository-specific quick start instructions.

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it TODO bash
    ```
1. Inside the container, launch the pre-built nodes.
    ```bash
    ros2 launch ackermann_trajectory_control ackermann_trajectory_control.launch.py
    ```

## 🧑‍💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://gitlab.ika.rwth-aachen.de/fb-fi/its-modules/control/ackermann_trajectory_control.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd ackermann_trajectory_control
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Build

> *Ctrl+Shift+B*

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

For further details see the respective package README files and the [Doxygen Documentation](https://openads-project.github.io/ackermann_trajectory_control).

| Package | Purpose |
| --- | --- |
| [ackermann_trajectory_control](ackermann_trajectory_control/README.md) | This package contains a trajectory controller for Ackermann-steered vehicles. It is implemented as a ROS 2 C++ node that subscribes to trajectory_planning_msgs/Trajectory and publishes control commands as ackermann_msgs/AckermannDriveStamped. |

## ⚖️ Licensing

- The source code in this repository is licensed under Apache-2.0. See [LICENSE](LICENSE).
- Docker images built from this repository also contain third-party software with its own license terms.

## 🙏 Acknowledgements

TODO: Project/funding acknowledgements
