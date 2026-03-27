// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

/**
 * @file AckermannTrajectoryControlNode.hpp
 * @brief Declares the Ackermann trajectory controller node and its helper utilities.
 */
#pragma once

#include <tracetools/tracetools.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <PID.hpp>

// Input Messages
#include <perception_msgs/msg/ego_data.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>

// Output Messages
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_trajectory_planning_msgs/tf2_trajectory_planning_msgs.hpp>

/**
 * @brief Type trait that evaluates to `true` for `std::vector` specializations.
 *
 * @tparam C Container type to inspect.
 */
template <typename C>
struct is_vector : std::false_type {};

/**
 * @brief Specialization that marks `std::vector` types as vectors.
 *
 * @tparam T Vector element type.
 * @tparam A Allocator type.
 */
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};

/**
 * @brief Convenience variable template for `is_vector`.
 *
 * @tparam C Container type to inspect.
 */
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief ROS 2 node that converts ego-state and trajectory input into Ackermann control commands.
 *
 * The node runs cascaded lateral and longitudinal PID controllers, applies configurable feed-forward terms, and
 * constrains the resulting curvature and acceleration commands before publishing them as Ackermann drive messages.
 */
class AckermannTrajectoryControl : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the trajectory controller node and initializes its ROS interfaces.
   */
  AckermannTrajectoryControl();

  /**
   * @brief Destroys the trajectory controller node.
   */
  ~AckermannTrajectoryControl();

 private:
  /**
   * @brief Creates publishers, subscriptions, timers, and controller instances.
   */
  void setup();

  /**
   * @brief Declares ROS parameters and loads their current values.
   */
  void loadParameters();

  /**
   * @brief Declares a parameter, loads its current value, and optionally enables runtime reconfiguration.
   *
   * @tparam T Parameter value type.
   * @param name Parameter name.
   * @param member_param Target member that stores the parameter value.
   * @param description Human-readable parameter description.
   * @param add_to_auto_reconfigurable_params Whether to update the member automatically on parameter changes.
   * @param is_required Whether the parameter must be initialized externally.
   * @param read_only Whether the parameter should be marked read-only.
   * @param from_value Optional minimum allowed value.
   * @param to_value Optional maximum allowed value.
   * @param step_value Optional discrete step size for ranged parameters.
   * @param additional_constraints Additional textual constraints shown in parameter metadata.
   */
  template <typename T>
  void declareAndLoadParameter(const std::string& name, T& member_param, const std::string& description,
                               const bool add_to_auto_reconfigurable_params = true, const bool is_required = false,
                               const bool read_only = false, const std::optional<T>& from_value = std::nullopt,
                               const std::optional<T>& to_value = std::nullopt,
                               const std::optional<T>& step_value = std::nullopt,
                               const std::string& additional_constraints = "");

  /**
   * @brief Applies runtime parameter updates.
   *
   * @param parameters Parameters that changed.
   * @return Result reported back to the ROS parameter infrastructure.
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Stores the latest ego-state message and refreshes the vehicle-frame trajectory.
   *
   * @param msg Received ego-state message.
   */
  void VehicleStateCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg);

  /**
   * @brief Stores the latest reference trajectory and probably resets controllers when high-level initialization is detected.
   *
   * @param msg Received trajectory message.
   */
  void TrajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  /**
   * @brief Callback to identify whether lateral control is active or not on the actuator side.
   *
   * @param msg Boolean control enable message.
   */
  void LatActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  /**
   * @brief Callback to identify whether longitudinal control is active or not on the actuator side.
   *
   * @param msg Boolean control enable message.
   */
  void LonActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  /**
   * @brief Executes one control-loop iteration and publishes the resulting Ackermann command. This function is periodically called by a ROS timer.
   */
  void VehicleCtrlCycle();

  /**
   * @brief Updates controller and feed-forward gains from the current vehicle speed.
   */
  void setControllerGains();

  /**
   * @brief Validates that current state and trajectory inputs are usable for control.
   *
   * @return `true` if all required inputs are valid.
   */
  bool InputSanityCheck();

  /**
   * @brief Extracts target values from the transformed trajectory and updates control deviations.
   *
   * @return `true` if all required target values were generated successfully.
   */
  bool TrjDataProc();

  /**
   * @brief Interpolates a sampled function at a requested x-position.
   *
   * @param X Sample positions.
   * @param Y Sample values.
   * @param desired_x Query position.
   * @param output_y Interpolated output value.
   * @return `true` if interpolation succeeded.
   */
  bool LinearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x,
                           double& output_y);

  /**
   * @brief Loads speed-dependent curvature limits from a CSV file.
   *
   * @return `true` if valid limit data was loaded.
   */
  bool LoadLateralLimitsCsv();

  /**
   * @brief Updates active curvature limits for the current longitudinal speed.
   *
   * @param velocity Current longitudinal velocity in meters per second.
   */
  void UpdateLateralLimitsFromVelocity(const double velocity);

  /**
   * @brief Integrates lateral and heading deviation from the measured vehicle motion.
   *
   * @param dt Integration step in seconds.
   */
  void CalcOdometry(const double dt);

  /**
   * @brief Resets the internally integrated lateral and heading odometry error.
   */
  void ResetOdometry();

  /**
   * @brief Checks whether the current ego-state sample is fresh enough to be used.
   *
   * @return `true` if the ego-state timestamp is within the configured timeout window.
   */
  bool VehicleStateOk() const;

  /**
   * @brief Derives the current curvature state from the measured steering angle. Also updates the cached curvature and curvature rate for the next control cycle.
   *
   * @return Current steering angle in radians.
   */
  double UpdateKappaFromState();

  /**
   * @brief Copies measured longitudinal state into the outgoing control message.
   */
  void UpdateLonFromState();

  /**
   * @brief Computes the target curvature for the current control step.
   *
   * @param dt Control-loop step size in seconds.
   * @return Target curvature in inverse meters.
   */
  double LateralControl(const double dt);

  /**
   * @brief Computes the target longitudinal acceleration for the current control step.
   *
   * @param dt Control-loop step size in seconds.
   * @return Target acceleration in meters per second squared.
   */
  double LongitudinalControl(const double dt);

  /**
   * @brief Resets controller state, cached outputs, and stored input messages.
   */
  void ResetController();

  /// Subscribers for ego-state, trajectory, and controller activation messages.
  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lat_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lon_active_sub_;

  /// Publisher for the computed Ackermann control command.
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr vehicle_ctrl_pub_;

  /// Periodic timer that drives the controller loop.
  rclcpp::TimerBase::SharedPtr vhcl_ctrl_timer_;

  /// Parameter callback handle used for runtime reconfiguration.
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  /// TF buffer and listener used to transform trajectories into the vehicle frame.
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  /// Latest subscribed state and trajectory data.
  perception_msgs::msg::EgoData cur_vehicle_state_;
  trajectory_planning_msgs::msg::Trajectory subscribed_trajectory_;
  trajectory_planning_msgs::msg::Trajectory tf_trajectory_;

  /// Timestamp of the most recent controller cycle.
  rclcpp::Time last_time_;

  /// Registered parameters that can be applied directly at runtime.
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter&)>>> auto_reconfigurable_params_;

  /// Frame and timing configuration.
  std::string vehicle_frame_id_ = "base_link";
  std::string fixed_over_time_frame_id_ = "map";
  double control_frequency_ = 100.0;

  /// Look-ahead times used for longitudinal and lateral target extraction.
  double lat_t_lookahead_ = 0.1;
  double lon_t_lookahead_ = 0.1;

  /// Longitudinal limits.
  double lon_max_acc_ = 3.5;
  double lon_min_acc_ = -5.0;  // make sure that this value is negative
  double lon_max_jerk_ = 5.0;

  /// Curvature limits and anti-windup configuration.
  double max_curvature_ = 0.0;
  double max_curvature_rate_ = 0.0;
  double max_curvature_accel_ = 0.0;
  double max_curvature_current_ = 0.0;
  double max_curvature_rate_current_ = 0.0;
  double anti_windup_gain_ = 1.0;
  bool use_back_calculation_ = false;
  bool use_speed_dependent_lateral_limits_ = false;
  bool lateral_limits_loaded_ = false;
  std::string lateral_limits_csv_path_ =
      ament_index_cpp::get_package_share_directory("ackermann_trajectory_control") + "/config/example-limits.csv";

  /// Maximum accepted ego-state age in seconds.
  double vehicle_state_timeout_ = 0.2;

  /// Vehicle model parameters.
  double wheelbase_ = 3.1;
  double self_st_gradient_ = 0.00265;

  /// Target values extracted from the reference trajectory.
  double a_tgt_ = 0.0;
  double a_tgt_dv_ = 0.0;
  double v_tgt_ = 0.0;
  double y_tgt_ = 0.0;
  double psi_tgt_ = 0.0;
  double delta_tgt_ = 0.0;

  /// Current control deviations and cached curvature state.
  double dpsi_ = 0.0;
  double dy_ = 0.0;
  double dv_ = 0.0;
  double last_kappa_ = 0.0;
  double last_kappa_rate_ = 0.0;
  bool lat_active_ = true;
  bool lon_active_ = true;

  /// PID controller instances for longitudinal speed, lateral error, and heading error.
  std::unique_ptr<PID> dv_pid_;
  std::unique_ptr<PID> dy_pid_;
  std::unique_ptr<PID> dpsi_pid_;

  /// Gain scheduling tables and optional speed-dependent curvature limits.
  std::vector<double> dv_p_ = {0.0, 0.0};
  std::vector<double> dv_i_ = {0.0, 0.0};
  std::vector<double> dv_d_ = {0.0, 0.0};
  std::vector<double> dy_p_ = {0.0, 0.0};
  std::vector<double> dy_i_ = {0.0, 0.0};
  std::vector<double> dy_d_ = {0.0, 0.0};
  std::vector<double> dpsi_p_ = {0.0, 0.0};
  std::vector<double> dpsi_i_ = {0.0, 0.0};
  std::vector<double> dpsi_d_ = {0.0, 0.0};
  std::vector<double> gain_scheduling_velocity_lookup_ = {-30.0, 30.0};  // velocity in m/s
  std::vector<double> lateral_limits_velocity_;
  std::vector<double> lateral_limits_kappa_max_;
  std::vector<double> lateral_limits_kappa_rate_max_;

  std::vector<double> vec_feed_forward_gain_steering_angle_ = {0.0, 0.0};
  std::vector<double> vec_feed_forward_gain_acceleration_ = {0.0, 0.0};

  /// Feed-forward gains selected for the current speed.
  double feed_forward_gain_steering_angle_ = 1.0;
  double feed_forward_gain_acceleration_ = 1.0;

  /// Most recently published control command.
  ackermann_msgs::msg::AckermannDriveStamped vhcl_ctrl_output_;

  /// Integrated lateral and heading deviations since the last trajectory transform.
  double odom_dy_ = 0.0;
  double odom_dpsi_ = 0.0;

};  // Class AckermannTrajectoryControl
