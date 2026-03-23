# `ackermann_trajectory_control`

This package contains a trajectory controller for ackermann steered vehicles. It is implemented as ROS 2 C++ node, which subscribes to a `trajectory_planning_msgs/Trajectory` and publishes the control commands as `ackermann_msgs/AckermannDriveStamped` to the vehicle.

- [Nodes](#nodes)
  - [ackermann_trajectory_controller](#ackermann_trajectory_controller)
- [Launch Files](#launch-files)

## Nodes

### `ackermann_trajectory_controller`

```mermaid
flowchart LR
    NODE("ackermann_trajectory_controller")
    S0:::hidden -->|~/ego_data| NODE
    S1:::hidden -->|~/trajectory| NODE
    S2:::hidden -->|~/lat_control_active| NODE
    S3:::hidden -->|~/lon_control_active| NODE
    NODE -->|~/controls| P0:::hidden
    classDef hidden display: none;
```

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ego_data` | `perception_msgs/msg/EgoData` | Ego data of the vehicle. Contains the current state (v, a, yaw, delta, etc.) of the vehicle |
| `~/trajectory` | `trajectory_planning_msgs/msg/Trajectory` | Trajectory to follow. Must be from type `DRIVABLE` |
| `~/lat_control_active` | `std_msgs/msg/Bool` | Indicates if lateral control is active |
| `~/lon_control_active` | `std_msgs/msg/Bool` | Indicates if longitudinal control is active |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/controls` | `ackermann_msgs/msg/AckermannDriveStamped` | Control commands for the vehicle. Longitudinal: acceleration, lateral: steering angle |

#### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `vehicle_frame_id` | `string` | `"base_link"` | Frame ID of the vehicle |
| `fixed_over_time_frame_id` | `string` | `"map"` | Frame ID of the fixed frame for time-wise transformations  (e.g. map) |
| `control_frequency` | `float` | `100.0` | Frequency of the control loop in Hz |
| `vehicle_state_timeout` | `float` | `0.2` | Maximum allowed age of the ego data in seconds |
| `wheelbase` | `float` | `2.711` | Wheelbase of the vehicle in meters (required for lateral control) |
| `selfsteergradient` | `float` | `0.002917853041365` | Self steer gradient of the vehicle (required for lateral control) |
| `longitudinal_lookahed_time` | `float` | `0.1` | Time in seconds for the longitudinal look ahead |
| `lateral_lookahed_time` | `float` | `0.1` | Time in seconds for the lateral look ahead |
| `max_longitudinal_acceleration` | `float` | `3.5` | Maximum allowed longitudinal acceleration in m/s^2 (constraint) |
| `min_longitudinal_acceleration` | `float` | `-5.0` | Minimum allowed longitudinal acceleration in m/s^2 (constraint, should be negative) |
| `max_longitudinal_jerk` | `float` | `5.0` | Maximum allowed longitudinal jerk in m/s^3 (constraint, absolute value) |
| `max_curvature` | `float` | `0.0` | Maximum allowed curvature (constraint, absolute value) |
| `max_curvature_rate` | `float` | `0.0` | Maximum allowed curvature rate (constraint, absolute value) |
| `max_curvature_acceleration` | `float` | `0.0` | Maximum allowed curvature acceleration (constraint, absolute value) |
| `use_speed_dependent_lateral_limits` | `bool` | `false` | Boolean to indicate if controllers uses speed dependent curvature limits given through a CSV file |
| `lateral_limits_csv` | `string` | `ament_index_cpp::get_package_share_directory("ackermann_trajectory_control") +
      "/config/example-limits.csv"` | CSV file path for speed dependent curvature limits |
| `anti_windup_gain` | `float` | `1.0` | Anti-windup back-calculation gain |
| `use_back_calculation` | `bool` | `false` | Enable anti-windup back-calculation |
| `velocity_lookup` | `float[]` | `[-30.0, 30.0]` | List of velocities in m/s for which the following gains are defined  |
| `feed_forward_acceleration_gain` | `float[]` | `[0.0, 0.0]` | List of feed forward gains for the acceleration controller (mapping to velocity_lookup) |
| `feed_forward_steering_angle_gain` | `float[]` | `[0.0, 0.0]` | List of feed forward gains for the steering angle controller (mapping to velocity_lookup) |
| `dv_p` | `float[]` | `[0.0, 0.0]` | List of proportional gains for the velocity controller (mapping to velocity_lookup) |
| `dv_i` | `float[]` | `[0.0, 0.0]` | List of integral gains for the velocity controller (mapping to velocity_lookup) |
| `dv_d` | `float[]` | `[0.0, 0.0]` | List of derivative gains for the velocity controller (mapping to velocity_lookup) |
| `dy_p` | `float[]` | `[0.0, 0.0]` | List of proportional gains for the lateral controller (mapping to velocity_lookup) |
| `dy_i` | `float[]` | `[0.0, 0.0]` | List of integral gains for the lateral controller (mapping to velocity_lookup) |
| `dy_d` | `float[]` | `[0.0, 0.0]` | List of derivative gains for the lateral controller (mapping to velocity_lookup) |
| `dpsi_p` | `float[]` | `[0.0, 0.0]` | List of proportional gains for the heading deviation controller (mapping to velocity_lookup) |
| `dpsi_i` | `float[]` | `[0.0, 0.0]` | List of integral gains for the heading deviation controller (mapping to velocity_lookup) |
| `dpsi_d` | `float[]` | `[0.0, 0.0]` | List of derivative gains for the heading deviation controller (mapping to velocity_lookup) |

## Launch Files

### [`ackermann_trajectory_control.launch.py`](launch/ackermann_trajectory_control.launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `ego_data_topic` | `"~/ego_data"` | topic for ego data input |
| `trajectory_topic` | `"~/trajectory"` | topic for trajectory input |
| `lat_control_active_topic` | `"~/lat_control_active"` | topic for lateral control activation |
| `lon_control_active_topic` | `"~/lon_control_active"` | topic for longitudinal control activation |
| `controls_topic` | `"~/controls"` | topic for control commands output |
| `name` | `"ackermann_trajectory_control_node"` | node name |
| `namespace` | `""` | node namespace |
| `params` | `os.path.join(get_package_share_directory("ackermann_trajectory_control"), "config", "params.yml")` | path to parameter file |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"false"` | use simulation clock |
| `trace` | `"False"` | Enable tracing |
