# Trajectory Control

The [`trajectory_control`](.) contains a sample C++ node [`TrajectoryControlNode`](src/TrajectoryControlNode.cpp) that supports the following ROS 2 functionality:
- [Trajectory Control](#trajectory-control)
    - [trajectory\_control/TrajectoryControlNode](#trajectory_controltrajectorycontrolnode)
      - [Subscribed Topics](#subscribed-topics)
      - [Published Topics](#published-topics)
      - [Services](#services)
      - [Actions](#actions)
      - [Parameters](#parameters)
  - [Usage of docker-ros Images](#usage-of-docker-ros-images)
    - [Available Images](#available-images)
    - [Default Command](#default-command)
    - [Environment Variables](#environment-variables)
    - [Launch Files](#launch-files)
    - [Configuration Files](#configuration-files)
    - [Additional Remarks](#additional-remarks)

### trajectory_control/TrajectoryControlNode

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `input_topic` | `sample_interfaces/msg/SampleMessage` | Echo the received message |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `output_topic` | `sample_interfaces/msg/SampleMessage` | Publish a `sample_interfaces::msg::SampleMessage` every `period` seconds |

#### Services

| Service | Type | Description |
| --- | --- | --- |
| `service` | `sample_interfaces/srv/SampleService` | Add two numbers |

#### Actions

| Action | Type | Description |
| --- | --- | --- |
| `action` | `sample_interfaces/action/SampleAction` | Compute Fibonnaci number |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `period` | `double` | **[dynamic]** Period of publishing to `output_topic` |
| `startup_state` | `int` | State according to [lifecycle documentation](#lifecycle) in which the node is started |


## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| `latest` | latest version |
| `latest-dev` | latest dev version |

### Default Command

```bash
ros2 launch sample_package_cpp sample_node_launch.py startup_state:=3
```

### Environment Variables

| Variable | Description |
| --- | --- |

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `sample_package_cpp` | `sample_node_launch.py` | `/docker-ros/ws/install/share/sample_package_cpp/launch` | Default launch file starting a node |
| `sample_package_cpp` | `sample_component_launch.py` | `/docker-ros/ws/install/share/sample_package_cpp/launch` | Default launch file starting a component in an internal or external container |

### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `sample_package_cpp` | `params.yml` | `/docker-ros/ws/install/share/sample_package_cpp/config` | Parameters for launch files |

### Additional Remarks

- 