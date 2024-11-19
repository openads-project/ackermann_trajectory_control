# Trajectory Control
This package contains a trajectory controller for ackermann steered vehicles. It is implemented as ROS 2 C++ node, which subscribes to a `trajectory_planning_msgs/Trajectory` and publishes the control commands as `ackermann_msgs/AckermannDrive` (lon: acceleration, lat: steering_angle) to the vehicle. 

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/input_trajectory` | `trajectory_planning_msgs/msg/Trajectory` | Trajectory to follow. Must be from type `DRIVABLE` |
| `~/input_ego_data` | `perception_msgs/msg/EgoData` | Ego data of the vehicle. Contains the current state (v, a, yaw, delta, etc.) of the vehicle |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ctrl_cmds` | `ackermann_msgs/msg/AckermannDrive` | Control commands for the vehicle. Longitudinal: acceleration, lateral: steering angle |

### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `vehicle_frame_id` | `string` | Frame id of the vehicle in which the trajectory should be handled |
| `fixed_over_time_frame_id` | `string` | Frame id of a fixed frame for time-wise transformations  (e.g. map) |
| `control_frequency` | `double` | Frequency of the control loop in Hz |
| `vehicle_state_timeout` | `double` | Maximum allowed age of the ego data in seconds |
| `lateral_lookahed_time` | `double` | Time in seconds for the lateral look ahead |
| `longitudinal_lookahed_time` | `double` | Time in seconds for the longitudinal look ahead |
| `wheelbase` | `double` | Wheelbase of the vehicle in meters (needed for lateral control) |
| `selfsteergradient` | `double` | Self steer gradient of the vehicle (needed for lateral control) |
| `max_longitudinal_acceleration` | `double` | Maximum allowed longitudinal acceleration in m/s^2 (constraint) |
| `min_longitudinal_acceleration` | `double` | Minimum allowed longitudinal acceleration in m/s^2 (constraint, should be negative) |
| `max_longitudinal_jerk` | `double` | Maximum allowed longitudinal jerk in m/s^3 (constraint, absolute value) |
| `max_steering_angle` | `double` | Maximum allowed steering angle in degrees (constraint, absolute value) |
| `max_steering_angle_rate` | `double` | Maximum allowed steering angle rate in degrees/s (constraint, absolute value) |
| `velocity_lookup` | `double[]` | List of velocities in m/s for which the following gains are defined |
| `feed_forward_acceleration_gain` | `double[]` | List of feed forward gains for the acceleration controller (mapping to `velocity_lookup`) |
| `feed_forward_steering_angle_gain` | `double[]` | List of feed forward gains for the steering angle controller (mapping to `velocity_lookup`) |
| `dv_p` | `double[]` | List of proportional gains for the velocity controller (mapping to `velocity_lookup`) |
| `dv_i` | `double[]` | List of integral gains for the velocity controller (mapping to `velocity_lookup`) |
| `dv_d` | `double[]` | List of derivative gains for the velocity controller (mapping to `velocity_lookup`) |
| `dy_p` | `double[]` | List of proportional gains for the lateral deviation controller (mapping to `velocity_lookup`) |
| `dy_i` | `double[]` | List of integral gains for the lateral deviation controller (mapping to `velocity_lookup`) |
| `dy_d` | `double[]` | List of derivative gains for the lateral deviation controller (mapping to `velocity_lookup`) |
| `dpsi_p` | `double[]` | List of proportional gains for the heading deviation controller (mapping to `velocity_lookup`) |
| `dpsi_i` | `double[]` | List of integral gains for the heading deviation controller (mapping to `velocity_lookup`) |
| `dpsi_d` | `double[]` | List of derivative gains for the heading deviation controller (mapping to `velocity_lookup`) |

### Usage of docker-ros Images

#### Available Images

| Tag | Description |
| --- | --- |
| `latest` | latest version |
| `latest-dev` | latest dev version |

#### Default Command

```bash
ros2 launch ackermann_trajectory_control ackermann_trajectory_control_node.launch.py
```

#### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `ackermann_trajectory_control` | `ackermann_trajectory_control_node.launch.py` | `/docker-ros/ws/install/ackermann_trajectory_control/share/ackermann_trajectory_control/launch/` | Default launch file starting a node |

#### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `ackermann_trajectory_control` | `params.yml` | `/docker-ros/ws/install/ackermann_trajectory_control/share/ackermann_trajectory_control/config` | Default set of parameters. Proven in simulation and Passat CC. |