// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <tracetools/tracetools.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
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

template <typename C>
struct is_vector : std::false_type {};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};

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
  struct CurvatureCommand {
    double kappa = 0.0;
    double kappa_rate = 0.0;
  };

  struct SteeringCommand {
    double steering_angle = 0.0;
    double steering_angle_rate = 0.0;
  };

  struct LongitudinalCommand {
    double speed = 0.0;
    double acceleration = 0.0;
    double jerk = 0.0;
  };

  /**
   * @brief Creates publishers, subscriptions, timers, and controller instances.
   */
  void setup();

  /**
   * @brief Declares a parameter, loads its current value, and optionally enables runtime reconfiguration.
   *
   * @tparam T Parameter value type.
   * @param name Parameter name.
   * @param param Target member that stores the parameter value.
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
  void declareAndLoadParameter(const std::string& name,
                               T& param,
                               const std::string& description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double>& from_value = std::nullopt,
                               const std::optional<double>& to_value = std::nullopt,
                               const std::optional<double>& step_value = std::nullopt,
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
  bool TrjDataProc(const rclcpp::Time& ctrl_time);

  /**
   * @brief Interpolates a sampled function at a requested x-position.
   *
   * @param X Sample positions.
   * @param Y Sample values.
   * @param desired_x Query position.
   * @param output_y Interpolated output value.
   * @return `true` if interpolation succeeded.
   */
  bool LinearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y);

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
  bool VehicleStateOk(const rclcpp::Time& ctrl_time) const;

  /**
   * @brief Derives the current steering command from the measured steering state and computes the matching curvature
   * state.
   *
   * The returned steering command contains measured steering-angle actual values, not controller target values.
   *
   * @param ego_data Current ego-state input.
   * @param wheelbase Wheelbase used for Ackermann conversion in meters.
   * @param kappa Output curvature computed from the measured steering angle.
   * @param kappa_rate Output curvature rate computed from the measured steering-angle rate.
   * @return Current measured steering command in radians and radians per second.
   */
  static SteeringCommand UpdateKappaFromState(const perception_msgs::msg::EgoData& ego_data,
                                              const double wheelbase,
                                              double& kappa,
                                              double& kappa_rate);

  /**
   * @brief Derives the current longitudinal command from the measured longitudinal state.
   *
   * The returned command contains measured speed and acceleration actual values. Jerk is set to `0.0` because the
   * ego-state input does not provide a measured longitudinal jerk.
   *
   * @param ego_data Current ego-state input.
   * @return Current measured longitudinal command in meters per second, meters per second squared, and meters per
   * second cubed.
   */
  static LongitudinalCommand UpdateLonFromState(const perception_msgs::msg::EgoData& ego_data);

  /**
   * @brief Applies curvature, curvature-rate, and curvature-acceleration limits to the requested target curvature.
   *
   * @param dt Control-loop step size in seconds.
   * @param kappa_tgt Requested target curvature, updated in-place when limited.
   * @param kappa_rate Resulting target curvature rate after limiting.
   * @param max_curvature Active curvature limit.
   * @param max_curvature_rate Active curvature-rate limit.
   * @param max_curvature_accel Active curvature-acceleration limit.
   * @param kappa_prev Previously commanded curvature.
   * @param kappa_rate_prev Previously commanded curvature rate.
   * @return `true` if any curvature limit became active.
   */
  bool LimitKappa(const double dt,
                  double& kappa_tgt,
                  double& kappa_rate,
                  const double max_curvature,
                  const double max_curvature_rate,
                  const double max_curvature_accel,
                  const double kappa_prev,
                  const double kappa_rate_prev);

  /**
   * @brief Converts curvature-domain commands into steering-domain commands for the current vehicle geometry.
   *
   * @param command Curvature command in inverse meters and inverse meters per second.
   * @return Steering command in radians and radians per second.
   */
  SteeringCommand CurvatureToSteeringCommand(const CurvatureCommand& command) const;

  /**
   * @brief Computes the target curvature for the current control step.
   *
   * @param dt Control-loop step size in seconds.
   * @return Target curvature command in inverse meters and inverse meters per second.
   */
  CurvatureCommand LateralControl(const double dt);

  /**
   * @brief Computes the target longitudinal command for the current control step.
   *
   * @param dt Control-loop step size in seconds.
   * @return Target longitudinal command in meters per second, meters per second squared, and meters per second cubed.
   */
  LongitudinalCommand LongitudinalControl(const double dt);

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

  /// Callback group separating the control-cycle timer from the default input/parameter group.
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;

  /// Mutexes protecting shared input cache and controller-internal state.
  std::mutex input_mutex_;
  std::mutex control_mutex_;

  /// TF buffer and listener used to transform trajectories into the vehicle frame.
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  /// Latest input cache shared between input callbacks and the control cycle.
  perception_msgs::msg::EgoData latest_vehicle_state_;
  trajectory_planning_msgs::msg::Trajectory latest_subscribed_trajectory_;
  bool latest_lat_active_ = true;
  bool latest_lon_active_ = true;
  uint64_t latest_trajectory_sequence_ = 0;
  uint64_t processed_trajectory_sequence_ = 0;

  /// Control-thread working copies of the subscribed state and trajectory data.
  perception_msgs::msg::EgoData cur_vehicle_state_;
  trajectory_planning_msgs::msg::Trajectory tf_trajectory_;

  /// Captured timestamp of the current control cycle.
  rclcpp::Time ctrl_time_;

  /// Timestamp of the most recent control-cycle callback.
  rclcpp::Time last_cycle_time_;

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
  bool use_odom_ = true;
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

  /// Other internal state variables
  double standstill_request_acceleration_gain_ = 0.0;

};  // Class AckermannTrajectoryControl
