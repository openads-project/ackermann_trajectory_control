# Cascaded Trajectory Control for Ackermann-Steered Vehicles

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
</p>

This repository contains a trajectory controller for Ackermann-steered vehicles. It is implemented as a ROS 2 C++ node that subscribes to [`trajectory_planning_msgs/Trajectory`](https://github.com/ika-rwth-aachen/planning_interfaces) and [`perception_msgs/EgoData`](https://github.com/ika-rwth-aachen/perception_interfaces) and publishes control commands as [`ackermann_msgs/AckermannDriveStamped`](https://github.com/ros-drivers/ackermann_msgs).

The control loop is executed at a configurable frequency and consists of a cascaded PID control architecture with a feedforward term based on the trajectory's curvature and acceleration and a feedback term based on velocity deviations for longitudinal control and lateral displacement and yaw-deviations for lateral control w.r.t. the planned trajectory. Additional features of the controller are:
- **Control Limiting**: The controller limits the control commands to user-defined maximum values for longitudinal acceleration and jerk, as well as curvature, curvature rate, and curvature acceleration for lateral control.
- **Anti-Windup**: The controller implements configurable anti-windup mechanisms for the integral term of the PID controller to prevent excessive accumulation of the integral error when the control commands are saturated.
- **Principle of Bi-Level-Stabilization**: The controller supports the principle of bi-level-stabilization according to [Werling](https://publikationen.bibliothek.kit.edu/1000021738).
- **Velocity-Dependent Gain-Scheduling**: PID gains are scheduled based on the current velocity of the vehicle to improve control performance across a wide range of operating conditions.

This controller is designed to be used in the context of the the *Open Automated Driving Stack* and well tested in combination with the [`trajectory_optimization`](https://github.com/openads-project/trajectory_optimization).

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
1. Inside the container, launch the pre-built node.
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
