# ackermann_trajectory_control

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/ackermann_trajectory_control/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/ackermann_trajectory_control"/></a>
  <a href="https://github.com/openads-project/ackermann_trajectory_control/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/ackermann_trajectory_control"/></a>
  <br>
  <a href="https://github.com/openads-project/ackermann_trajectory_control/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/ackermann_trajectory_control/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/ackermann_trajectory_control/actions/workflows/compose-oci.yml"><img src="https://github.com/openads-project/ackermann_trajectory_control/actions/workflows/compose-oci.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/ackermann_trajectory_control"><img src="https://github.com/openads-project/ackermann_trajectory_control/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/ackermann_trajectory_control/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/ackermann_trajectory_control/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**Cascaded ROS 2 PID Controller for Ackermann steered vehicles**

This repository contains a trajectory controller for Ackermann-steered vehicles. It is implemented as a ROS 2 C++ node that subscribes to [`trajectory_planning_msgs/Trajectory`](https://github.com/ika-rwth-aachen/planning_interfaces) and [`perception_msgs/EgoData`](https://github.com/ika-rwth-aachen/perception_interfaces) and publishes control commands as [`ackermann_msgs/AckermannDriveStamped`](https://github.com/ros-drivers/ackermann_msgs).

The control loop is executed at a configurable frequency and consists of a cascaded PID control architecture with a feedforward term based on the trajectory's curvature and acceleration and a feedback term based on velocity deviations for longitudinal control and lateral displacement and yaw-deviations for lateral control w.r.t. the planned trajectory. Additional features of the controller are:
- **Control Limiting**: The controller limits the control commands to user-defined maximum values for longitudinal acceleration and jerk, as well as curvature, curvature rate, and curvature acceleration for lateral control.
- **Anti-Windup**: The controller implements configurable anti-windup mechanisms for the integral term of the PID controller to prevent excessive accumulation of the integral error when the control commands are saturated.
- **Principle of Bi-Level-Stabilization**: The controller supports the principle of bi-level-stabilization according to [Werling](https://publikationen.bibliothek.kit.edu/1000021738).
- **Velocity-Dependent Gain-Scheduling**: PID gains are scheduled based on the current velocity of the vehicle to improve control performance across a wide range of operating conditions.

This controller is designed to be used in the context of the the *Open Automated Driving Stack* and particularly in combination with the [`trajectory_optimization`](https://github.com/openads-project/trajectory_optimization).

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>


> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://openads-project.github.io/), the *Open Automated Driving Systems* project. *OpenADS* and its modules have been initiated and are currently being maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/de/).


## 🚀 Quick Start

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it ghcr.io/openads-project/ackermann_trajectory_control:latest bash
    ```
1. Inside the container, launch the pre-built node.
    ```bash
    ros2 launch ackermann_trajectory_control ackermann_trajectory_control.launch.py
    ```

## 💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/openads-project/ackermann_trajectory_control.git
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

Package and node interfaces are documented in the respective package READMEs listed below. Implementation details are found in the [Source Code Documentation](https://openads-project.github.io/ackermann_trajectory_control).

| Package | Description |
| --- | --- |
| [ackermann_trajectory_control](ackermann_trajectory_control/README.md) | This package contains a trajectory controller for Ackermann-steered vehicles. It is implemented as a ROS 2 C++ node that subscribes to trajectory_planning_msgs/Trajectory and publishes control commands as ackermann_msgs/AckermannDriveStamped. |

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

Development and maintenance of this repository are supported by the following projects. We acknowledge the funding of the respective institutions.

| Project | Funding Institution | Grant Number |
| --- | --- | --- |
| [AIthena](https://aithena.eu/) | 🇪🇺 European Union | 101076754 |

<p>
  <img src="https://ec.europa.eu/regional_policy/images/information-sources/logo-download-center/eu_funded_en.jpg" height=70>
</p>

<sup><sup>Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor CINEA can be held responsible for them.</sup></sup>
