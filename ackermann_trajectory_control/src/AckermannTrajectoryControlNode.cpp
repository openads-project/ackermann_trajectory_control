// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include "AckermannTrajectoryControlNode.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

AckermannTrajectoryControl::AckermannTrajectoryControl() : Node("ackermann_trajectory_controller") {
  // declare and load node parameters
  this->declareAndLoadParameter("vehicle_frame_id", vehicle_frame_id_, "Frame ID of the vehicle", false);
  this->declareAndLoadParameter("fixed_over_time_frame_id", fixed_over_time_frame_id_,
                                "Frame ID of the fixed frame used for transformations over time (e.g. map)", false);
  this->declareAndLoadParameter("control_frequency", control_frequency_, "Frequency of the control loop in Hz", true, false,
                                false, 0.0, 200.0, 1.0);
  this->declareAndLoadParameter("vehicle_state_timeout", vehicle_state_timeout_, "Maximum allowed age of the ego data in seconds",
                                true, false, false, 0.0, 1.0, 0.05);
  this->declareAndLoadParameter("wheelbase", wheelbase_, "Wheelbase of the vehicle in meters (required for lateral control)",
                                false);
  this->declareAndLoadParameter("selfsteergradient", self_st_gradient_,
                                "Self-steer gradient of the vehicle (required for lateral control)", false);
  this->declareAndLoadParameter("longitudinal_lookahead_time", lon_t_lookahead_,
                                "Time in seconds for the longitudinal look-ahead", true, false, false, 0.0, 5.0, 0.1);
  this->declareAndLoadParameter("lateral_lookahead_time", lat_t_lookahead_, "Time in seconds for the lateral look-ahead", true,
                                false, false, 0.0, 5.0, 0.1);
  this->declareAndLoadParameter("max_longitudinal_acceleration", lon_max_acc_,
                                "Maximum allowed longitudinal acceleration in m/s^2 (constraint)", true, false, false, 0.0, 10.0,
                                0.1);
  this->declareAndLoadParameter("min_longitudinal_acceleration", lon_min_acc_,
                                "Minimum allowed longitudinal acceleration in m/s^2 (constraint, should be negative)", true,
                                false, false, -10.0, 0.0, 0.1);
  this->declareAndLoadParameter("max_longitudinal_jerk", lon_max_jerk_,
                                "Maximum allowed longitudinal jerk in m/s^3 (constraint, absolute value)", true, false, false,
                                0.0, 20.0, 0.1);
  this->declareAndLoadParameter("max_curvature", max_curvature_, "Maximum allowed curvature (constraint, absolute value)", true,
                                false, false, 0.0, 1.0, 1e-12);
  this->declareAndLoadParameter("max_curvature_rate", max_curvature_rate_,
                                "Maximum allowed curvature rate (constraint, absolute value)", true, false, false, 0.0, 5.0,
                                1e-12);
  this->declareAndLoadParameter("max_curvature_acceleration", max_curvature_accel_,
                                "Maximum allowed curvature acceleration (constraint, absolute value)", true, false, false, 0.0,
                                20.0, 1e-12);
  this->declareAndLoadParameter("use_speed_dependent_lateral_limits", use_speed_dependent_lateral_limits_,
                                "Boolean indicating whether the controller uses speed-dependent curvature limits from a CSV file",
                                false);
  this->declareAndLoadParameter("lateral_limits_csv", lateral_limits_csv_path_,
                                "CSV file path for speed-dependent curvature limits", false);
  this->declareAndLoadParameter("anti_windup_gain", anti_windup_gain_, "Anti-windup back-calculation gain", true, false, false,
                                0.0, 100.0, 0.1);
  this->declareAndLoadParameter("use_back_calculation", use_back_calculation_, "Enable anti-windup back-calculation", true);
  this->declareAndLoadParameter("use_odom", use_odom_, "Enable controller-internal odometry integration", true);
  this->declareAndLoadParameter("velocity_lookup", gain_scheduling_velocity_lookup_,
                                "List of velocities in m/s for which the following gains are defined", false);
  this->declareAndLoadParameter("feed_forward_acceleration_gain", vec_feed_forward_gain_acceleration_,
                                "List of feed-forward gains for the acceleration controller (mapping to velocity_lookup)", true);
  this->declareAndLoadParameter("feed_forward_steering_angle_gain", vec_feed_forward_gain_steering_angle_,
                                "List of feed-forward gains for the steering-angle controller (mapping to velocity_lookup)",
                                true);
  this->declareAndLoadParameter("dv_p", dv_p_,
                                "List of proportional gains for the velocity controller (mapping to velocity_lookup)", true);
  this->declareAndLoadParameter("dv_i", dv_i_, "List of integral gains for the velocity controller (mapping to velocity_lookup)",
                                true);
  this->declareAndLoadParameter("dv_d", dv_d_,
                                "List of derivative gains for the velocity controller (mapping to velocity_lookup)", true);
  this->declareAndLoadParameter("dy_p", dy_p_,
                                "List of proportional gains for the lateral controller (mapping to velocity_lookup)", true);
  this->declareAndLoadParameter("dy_i", dy_i_, "List of integral gains for the lateral controller (mapping to velocity_lookup)",
                                true);
  this->declareAndLoadParameter("dy_d", dy_d_, "List of derivative gains for the lateral controller (mapping to velocity_lookup)",
                                true);
  this->declareAndLoadParameter(
      "dpsi_p", dpsi_p_, "List of proportional gains for the heading deviation controller (mapping to velocity_lookup)", true);
  this->declareAndLoadParameter("dpsi_i", dpsi_i_,
                                "List of integral gains for the heading deviation controller (mapping to velocity_lookup)", true);
  this->declareAndLoadParameter(
      "dpsi_d", dpsi_d_, "List of derivative gains for the heading deviation controller (mapping to velocity_lookup)", true);
  this->declareAndLoadParameter(
      "standstill_request_acceleration_gain", standstill_request_acceleration_gain_,
      "Gain for the deceleration request when the input trajectory signals standstill. Speed and jerk commands stay zero; "
      "acceleration is calculated from current speed and clamped by min_longitudinal_acceleration.",
      true, false, false, -5.0, 0.0, 0.1);

  this->setup();
}

AckermannTrajectoryControl::~AckermannTrajectoryControl() {}

AckermannTrajectoryControl::SteeringCommand AckermannTrajectoryControl::CurvatureToSteeringCommand(
    const CurvatureCommand& command) const {
  SteeringCommand steering_command;
  steering_command.steering_angle = std::atan(command.kappa * wheelbase_);
  if (!std::isfinite(command.kappa) || !std::isfinite(command.kappa_rate)) {
    return steering_command;
  }
  const double cos_steering_angle = std::cos(steering_command.steering_angle);
  steering_command.steering_angle_rate = command.kappa_rate * wheelbase_ * cos_steering_angle * cos_steering_angle;
  return steering_command;
}

template <typename T>
void AckermannTrajectoryControl::declareAndLoadParameter(const std::string& name,
                                                         T& param,
                                                         const std::string& description,
                                                         const bool add_to_auto_reconfigurable_params,
                                                         const bool is_required,
                                                         const bool read_only,
                                                         const std::optional<double>& from_value,
                                                         const std::optional<double>& to_value,
                                                         const std::optional<double>& step_value,
                                                         const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr (is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

rcl_interfaces::msg::SetParametersResult AckermannTrajectoryControl::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  std::lock_guard<std::mutex> control_lock(control_mutex_);

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        const bool old_use_odom = use_odom_;
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(),
                    param.value_to_string().c_str());
        if (param.get_name() == "use_odom" && use_odom_ != old_use_odom) {
          ResetOdometry();
        }
        break;
      }
    }
    if (param.get_name() == "max_curvature") {
      if (!use_speed_dependent_lateral_limits_) {
        max_curvature_current_ = max_curvature_;
      }
    }
    if (param.get_name() == "max_curvature_rate") {
      if (!use_speed_dependent_lateral_limits_) {
        max_curvature_rate_current_ = max_curvature_rate_;
      }
    }
  }

  // mark parameter change successful
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

void AckermannTrajectoryControl::setup() {
  // initialize dv-PID
  dv_pid_ = std::make_unique<PID>(0.0, 0.0, 0.0);

  // initialize dy-PID
  dy_pid_ = std::make_unique<PID>(0.0, 0.0, 0.0);

  // initialize dpsi-PID
  dpsi_pid_ = std::make_unique<PID>(0.0, 0.0, 0.0);

  // set Initial output values
  vhcl_ctrl_output_.drive.steering_angle = 0.0;
  vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
  vhcl_ctrl_output_.drive.speed = 0.0;
  vhcl_ctrl_output_.drive.acceleration = 0.0;
  vhcl_ctrl_output_.drive.jerk = 0.0;
  vhcl_ctrl_output_.header.stamp = this->now();

  ResetOdometry();

  // initialize tf listener
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  control_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // initialize subscribers
  vehicle_state_sub_ = create_subscription<perception_msgs::msg::EgoData>(
      "~/ego_data", 1, std::bind(&AckermannTrajectoryControl::VehicleStateCallback, this, std::placeholders::_1));
  trajectory_sub_ = create_subscription<trajectory_planning_msgs::msg::Trajectory>(
      "~/trajectory", 1, std::bind(&AckermannTrajectoryControl::TrajectoryCallback, this, std::placeholders::_1));
  lat_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/lat_control_active", 1, std::bind(&AckermannTrajectoryControl::LatActiveCallback, this, std::placeholders::_1));
  lon_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/lon_control_active", 1, std::bind(&AckermannTrajectoryControl::LonActiveCallback, this, std::placeholders::_1));

  // initialize publishers
  vehicle_ctrl_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("~/controls", 1);

  if (use_speed_dependent_lateral_limits_) {
    lateral_limits_loaded_ = LoadLateralLimitsCsv();
    if (!lateral_limits_loaded_) {
      RCLCPP_WARN_STREAM(get_logger(), "Speed dependent lateral limits disabled due to CSV load failure.");
    }
  }

  // Initialize active curvature limits from the configured static or speed-dependent constraints.
  UpdateLateralLimitsFromVelocity(0.0);

  // initialize the cyclic vehicle-control timer; the callback VehicleCtrlCycle will be called wrt. the defined control frequency
  last_cycle_time_ = now();
  vhcl_ctrl_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_frequency_),
                                       std::bind(&AckermannTrajectoryControl::VehicleCtrlCycle, this), control_callback_group_);

  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&AckermannTrajectoryControl::parametersCallback, this, std::placeholders::_1));

  // Annotate message links for tracing: Publish ackermann commands periodically, depending an all subscribed topics.
  std::vector<const void*> link_subs;
  link_subs.push_back(static_cast<const void*>(vehicle_state_sub_->get_subscription_handle().get()));
  link_subs.push_back(static_cast<const void*>(trajectory_sub_->get_subscription_handle().get()));
  std::vector<const void*> link_pubs;
  link_pubs.push_back(static_cast<const void*>(vehicle_ctrl_pub_->get_publisher_handle().get()));
  TRACETOOLS_TRACEPOINT(message_link_periodic_async, link_subs.data(), link_subs.size(), link_pubs.data(), link_pubs.size());

  // setup diagnostic updater
  diagnostic_updater_.setHardwareID(this->get_name());
  diagnostic_updater_.add("Health", this, &AckermannTrajectoryControl::health);
  setHealth(diagnostic_msgs::msg::DiagnosticStatus::STALE, "AckermannTrajectoryControl initialized", {{}});
  diagnostic_updater_.force_update();
}

void AckermannTrajectoryControl::health(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  stat.summary(health_.status, health_.message);
  for (const auto& [key, value] : health_.key_value_pairs) {
    stat.add(key, value);
  }
}

void AckermannTrajectoryControl::setHealth(const unsigned char status,
                                           const std::string& msg,
                                           const std::map<std::string, std::string>& key_value_pairs) {
  health_.status = status;
  health_.message = msg;
  health_.key_value_pairs = key_value_pairs;
}

void AckermannTrajectoryControl::VehicleStateCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> input_lock(input_mutex_);
  latest_vehicle_state_ = *msg;
}

void AckermannTrajectoryControl::TrajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> input_lock(input_mutex_);
  latest_subscribed_trajectory_ = *msg;
  ++latest_trajectory_sequence_;
}

void AckermannTrajectoryControl::LatActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> input_lock(input_mutex_);
  latest_lat_active_ = msg->data;
}

void AckermannTrajectoryControl::LonActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> input_lock(input_mutex_);
  latest_lon_active_ = msg->data;
}

void AckermannTrajectoryControl::ResetController() {
  a_tgt_ = 0.0;
  a_tgt_dv_ = 0.0;
  v_tgt_ = 0.0;
  y_tgt_ = 0.0;
  psi_tgt_ = 0.0;
  delta_tgt_ = 0.0;
  dpsi_ = 0.0;
  dy_ = 0.0;
  dv_ = 0.0;
  last_kappa_ = 0.0;
  last_kappa_rate_ = 0.0;
  dy_pid_->Reset();
  dpsi_pid_->Reset();
  dv_pid_->Reset();
  vhcl_ctrl_output_.drive.steering_angle = 0.0;
  vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
  vhcl_ctrl_output_.drive.speed = 0.0;
  vhcl_ctrl_output_.drive.acceleration = 0.0;
  vhcl_ctrl_output_.drive.jerk = 0.0;
  vhcl_ctrl_output_.header.stamp = this->now();
  ResetOdometry();
}

void AckermannTrajectoryControl::setControllerGains() {
  double velocity = perception_msgs::object_access::getVelLon(cur_vehicle_state_);
  // feed-forward gains
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, vec_feed_forward_gain_acceleration_, velocity,
                           feed_forward_gain_acceleration_)) {
    feed_forward_gain_acceleration_ = 0.0;
  }
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, vec_feed_forward_gain_steering_angle_, velocity,
                           feed_forward_gain_steering_angle_)) {
    feed_forward_gain_steering_angle_ = 0.0;
  }

  double p = 0.0, i = 0.0, d = 0.0;
  // dv Controller
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dv_p_, velocity, p)) p = 0.0;
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dv_i_, velocity, i)) i = 0.0;
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dv_d_, velocity, d)) d = 0.0;
  dv_pid_->SetParameters(p, i, d);

  // dy Controller
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dy_p_, velocity, p)) p = 0.0;
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dy_i_, velocity, i)) i = 0.0;
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dy_d_, velocity, d)) d = 0.0;
  dy_pid_->SetParameters(p, i, d);

  // dpsi Controller
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dpsi_p_, velocity, p)) p = 0.0;
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dpsi_i_, velocity, i)) i = 0.0;
  if (!LinearInterpolation(gain_scheduling_velocity_lookup_, dpsi_d_, velocity, d)) d = 0.0;
  dpsi_pid_->SetParameters(p, i, d);
}

void AckermannTrajectoryControl::VehicleCtrlCycle() {
  perception_msgs::msg::EgoData latest_vehicle_state;
  trajectory_planning_msgs::msg::Trajectory latest_subscribed_trajectory;
  bool latest_lat_active = true;
  bool latest_lon_active = true;
  uint64_t latest_trajectory_sequence = 0;

  {
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    latest_vehicle_state = latest_vehicle_state_;
    latest_subscribed_trajectory = latest_subscribed_trajectory_;
    latest_lat_active = latest_lat_active_;
    latest_lon_active = latest_lon_active_;
    latest_trajectory_sequence = latest_trajectory_sequence_;
  }

  std::lock_guard<std::mutex> control_lock(control_mutex_);
  ctrl_time_ = now();
  if (last_cycle_time_ > ctrl_time_) {
    RCLCPP_WARN_STREAM(get_logger(), "Resetting controller because of Jump-Back in time!");
    ResetController();
    vhcl_ctrl_output_.header.stamp = ctrl_time_;
  }

  const double cycle_dt = (ctrl_time_ - last_cycle_time_).seconds();
  if (cycle_dt > 1.25 / control_frequency_) {
    RCLCPP_WARN_STREAM(get_logger(), "Exceeding the configured cycle period! dt since last timer cycle: "
                                         << std::fixed << std::setprecision(15) << cycle_dt << " seconds.");
  }
  last_cycle_time_ = ctrl_time_;

  cur_vehicle_state_ = latest_vehicle_state;

  if (latest_trajectory_sequence != processed_trajectory_sequence_) {
    processed_trajectory_sequence_ = latest_trajectory_sequence;
    // check needs to be performed before any transformation because x=0, y=0, theta=0 is indicating a high-level-initialization
    if (trajectory_planning_msgs::trajectory_access::getSamplePointSize(latest_subscribed_trajectory) > 0) {
      // get x, y and theta of trajectory at first state
      double x = trajectory_planning_msgs::trajectory_access::getX(latest_subscribed_trajectory, 0);
      double y = trajectory_planning_msgs::trajectory_access::getY(latest_subscribed_trajectory, 0);
      double theta = trajectory_planning_msgs::trajectory_access::getTheta(latest_subscribed_trajectory, 0);
      if (x == 0.0 && y == 0.0 && theta == 0.0) {  // high-level-initialization
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        dv_pid_->Reset();
      }
    }
  }

  if (VehicleStateOk(ctrl_time_)) {
    // transform latest trajectory to current vehicle-frame
    try {
      tf_trajectory_ = tf2_buffer_->transform(latest_subscribed_trajectory, vehicle_frame_id_,
                                              tf2_ros::fromMsg(cur_vehicle_state_.header.stamp), fixed_over_time_frame_id_,
                                              tf2::durationFromSec(0.01));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Failed transforming trajectory in control cycle. Ex: %s", ex.what());
      tf_trajectory_ = latest_subscribed_trajectory;
    }
    ResetOdometry();
    UpdateLateralLimitsFromVelocity(perception_msgs::object_access::getVelLon(cur_vehicle_state_));
  }
  if (!latest_lat_active) {
    dy_pid_->Reset();
    dpsi_pid_->Reset();
  }
  if (!latest_lon_active) {
    dv_pid_->Reset();
  }
  if (!InputSanityCheck())  // some inputs are not ok
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Input sanity check failed! Skipping control cycle...");
    diagnostic_updater_.force_update();
    return;
  }

  if (!TrjDataProc(ctrl_time_)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Processing of input Trajectory failed! Skipping control cycle...");
    diagnostic_updater_.force_update();
    return;
  }

  double dt = (ctrl_time_ - vhcl_ctrl_output_.header.stamp).seconds();
  if (dt <= 0.0) {
    RCLCPP_ERROR_STREAM(get_logger(), "dt since last control output: " << std::fixed << std::setprecision(15) << dt
                                                                       << " seconds. Skipping control cycle...");
    diagnostic_updater_.force_update();
    return;
  } else if (dt > 1.25 / control_frequency_) {
    RCLCPP_WARN_STREAM(get_logger(), "Exceeding the expected dt since last control output! dt since last control output: "
                                         << std::fixed << std::setprecision(15) << dt << " seconds.");
  }
  // Hold zero output (except delta) while the trajectory explicitly requests standstill.
  if (tf_trajectory_.standstill) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Standstill.");
    CurvatureCommand curvature_command{std::tan(delta_tgt_) / wheelbase_, 0.0};
    LimitKappa(dt, curvature_command.kappa, curvature_command.kappa_rate, max_curvature_current_, max_curvature_rate_current_,
               max_curvature_accel_, last_kappa_, last_kappa_rate_);
    const SteeringCommand steering_command = CurvatureToSteeringCommand(curvature_command);
    vhcl_ctrl_output_.drive.steering_angle = static_cast<float>(steering_command.steering_angle);
    vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
    vhcl_ctrl_output_.drive.speed = 0.0;
    double standstill_request_acceleration =
        standstill_request_acceleration_gain_ * std::fabs(perception_msgs::object_access::getVelLon(cur_vehicle_state_));
    standstill_request_acceleration = std::min(standstill_request_acceleration, 0.0);
    standstill_request_acceleration = std::max(standstill_request_acceleration, lon_min_acc_);
    vhcl_ctrl_output_.drive.acceleration = static_cast<float>(standstill_request_acceleration);
    vhcl_ctrl_output_.drive.jerk = 0.0;
    dy_pid_->Reset();
    dpsi_pid_->Reset();
    dv_pid_->Reset();
    last_kappa_ = curvature_command.kappa;
    last_kappa_rate_ = curvature_command.kappa_rate;
  } else {
    setControllerGains();
    if (latest_lat_active) {
      const CurvatureCommand curvature_command = LateralControl(dt);
      const SteeringCommand steering_command = CurvatureToSteeringCommand(curvature_command);
      if (std::isnan(steering_command.steering_angle)) {
        RCLCPP_ERROR_STREAM(get_logger(), "Steering Angle Output Value isNaN! Skipping control cycle...");
        vhcl_ctrl_output_.drive.steering_angle = 0.0;
        vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        diagnostic_updater_.force_update();
        return;
      }
      vhcl_ctrl_output_.drive.steering_angle = static_cast<float>(steering_command.steering_angle);
      vhcl_ctrl_output_.drive.steering_angle_velocity = static_cast<float>(steering_command.steering_angle_rate);
    } else {
      const SteeringCommand steering_command =
          UpdateKappaFromState(cur_vehicle_state_, wheelbase_, last_kappa_, last_kappa_rate_);
      // use measured steering angle as output if lateral control is inactive
      vhcl_ctrl_output_.drive.steering_angle = static_cast<float>(steering_command.steering_angle);
      vhcl_ctrl_output_.drive.steering_angle_velocity = static_cast<float>(steering_command.steering_angle_rate);
    }
    if (latest_lon_active) {
      const LongitudinalCommand longitudinal_command = LongitudinalControl(dt);
      vhcl_ctrl_output_.drive.speed = static_cast<float>(longitudinal_command.speed);
      if (std::isnan(longitudinal_command.acceleration) || std::isnan(longitudinal_command.jerk)) {
        RCLCPP_ERROR_STREAM(get_logger(), "Target Longitudinal Output Value isNaN! Skipping control cycle...");
        vhcl_ctrl_output_.drive.acceleration = 0.0;
        vhcl_ctrl_output_.drive.jerk = 0.0;
        dv_pid_->Reset();
        diagnostic_updater_.force_update();
        return;
      } else {
        vhcl_ctrl_output_.drive.acceleration = static_cast<float>(longitudinal_command.acceleration);
        vhcl_ctrl_output_.drive.jerk = static_cast<float>(longitudinal_command.jerk);
      }
    } else {
      const LongitudinalCommand longitudinal_command = UpdateLonFromState(cur_vehicle_state_);
      vhcl_ctrl_output_.drive.speed = static_cast<float>(longitudinal_command.speed);
      vhcl_ctrl_output_.drive.acceleration = static_cast<float>(longitudinal_command.acceleration);
      vhcl_ctrl_output_.drive.jerk = static_cast<float>(longitudinal_command.jerk);
    }
  }
  vhcl_ctrl_output_.header.stamp = ctrl_time_;
  vehicle_ctrl_pub_->publish(vhcl_ctrl_output_);
  diagnostic_updater_.force_update();
}

bool AckermannTrajectoryControl::InputSanityCheck() {
  if (!VehicleStateOk(ctrl_time_)) {
    RCLCPP_ERROR_STREAM(get_logger(), "EgoState-Data outdated or invalid!");
    return false;
  }
  if (trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_) == 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Input trajectory is empty!");
    RCLCPP_DEBUG_STREAM(get_logger(), "Number of samples in tf_trajectory_: "
                                          << trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_));
    RCLCPP_DEBUG_STREAM(get_logger(), "Stamp of tf_trajectory_: " << tf_trajectory_.header.stamp.sec << "s "
                                                                  << tf_trajectory_.header.stamp.nanosec << "ns");
    return false;
  } else {
    // get last state of trajectory
    double last_time = trajectory_planning_msgs::trajectory_access::getT(
        tf_trajectory_, trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_) - 1);
    double lookahead = std::max(lon_t_lookahead_, lat_t_lookahead_);
    if (last_time < 2 * lookahead) {
      RCLCPP_ERROR_STREAM(get_logger(), "Input trajectory is too short!");
      return false;
    }
  }
  return true;
}

bool AckermannTrajectoryControl::TrjDataProc(const rclcpp::Time& ctrl_time) {
  // Derive State Vectors
  std::vector<double> TIME, V, A, Y, THETA, DELTA;
  int n_samples = trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_);
  for (int i = 0; i < n_samples; i++) {
    TIME.push_back(trajectory_planning_msgs::trajectory_access::getT(tf_trajectory_, i));
    V.push_back(trajectory_planning_msgs::trajectory_access::getV(tf_trajectory_, i));
    Y.push_back(trajectory_planning_msgs::trajectory_access::getY(tf_trajectory_, i));
    if (tf_trajectory_.type_id == trajectory_planning_msgs::DRIVABLE::TYPE_ID) {
      A.push_back(trajectory_planning_msgs::trajectory_access::getA(tf_trajectory_, i));
      THETA.push_back(trajectory_planning_msgs::trajectory_access::getTheta(tf_trajectory_, i));
      DELTA.push_back(trajectory_planning_msgs::trajectory_access::getDeltaAck(tf_trajectory_, i));
    } else {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Unsupported trajectory type. Only trajectory_planning_msgs::DRIVABLE is currently supported.");
      return false;
    }
  }

  // calculate desired interpolation time for longitudinal values
  double delta_time = (ctrl_time - tf_trajectory_.header.stamp).seconds();
  // interpolate longitudinal target values
  if (!LinearInterpolation(TIME, V, delta_time + lon_t_lookahead_, v_tgt_)) return false;
  if (!LinearInterpolation(TIME, A, delta_time + lon_t_lookahead_, a_tgt_)) return false;

  // interpolate lateral target values
  if (!LinearInterpolation(TIME, Y, delta_time + lat_t_lookahead_, y_tgt_)) return false;
  if (!LinearInterpolation(TIME, THETA, delta_time + lat_t_lookahead_, psi_tgt_)) return false;
  if (!LinearInterpolation(TIME, DELTA, delta_time + lat_t_lookahead_, delta_tgt_)) return false;

  if (use_odom_) {
    // CalcOdometry
    double dt_ego = (ctrl_time - cur_vehicle_state_.header.stamp).seconds();
    double dt_ctrl = (ctrl_time - vhcl_ctrl_output_.header.stamp).seconds();
    double dt = std::min(dt_ego, dt_ctrl);
    CalcOdometry(dt);  // Cyclic Control

    dy_ = odom_dy_ - y_tgt_;
    dpsi_ = odom_dpsi_ - psi_tgt_;
  } else {
    dy_ = -y_tgt_;
    dpsi_ = -psi_tgt_;
  }
  return true;
}

bool AckermannTrajectoryControl::LinearInterpolation(const std::vector<double>& X,
                                                     const std::vector<double>& Y,
                                                     const double& desired_x,
                                                     double& output_y) {
  if (desired_x < *min_element(X.begin(), X.end()) || desired_x > *max_element(X.begin(), X.end())) {
    RCLCPP_ERROR_STREAM(get_logger(), "Desired x value is outside the range covered by the input vector.");
    return false;
  }
  if (X.size() != Y.size()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Input vectors don't have the same length!");
    return false;
  }

  // go through array and search for sampling points
  size_t i = 0;
  for (i = 0; i < X.size(); i++) {
    if (X[i] < desired_x) {
      continue;
    } else if (X[i] == desired_x) {
      output_y = Y[i];
      return true;
    } else {
      break;
    }
  }
  output_y = Y[i - 1] + ((Y[i] - Y[i - 1]) / (X[i] - X[i - 1])) * (desired_x - X[i - 1]);
  return true;
}

bool AckermannTrajectoryControl::LoadLateralLimitsCsv() {
  lateral_limits_velocity_.clear();
  lateral_limits_kappa_max_.clear();
  lateral_limits_kappa_rate_max_.clear();

  if (lateral_limits_csv_path_.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "CSV path for lateral limits is empty.");
    return false;
  }

  std::string csv_path = lateral_limits_csv_path_;
  if (!csv_path.empty() && csv_path.front() != '/') {
    try {
      std::string share_dir = ament_index_cpp::get_package_share_directory("ackermann_trajectory_control");
      csv_path = share_dir + "/" + csv_path;
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to resolve CSV path: " << ex.what());
      return false;
    }
  }

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to open CSV file: " << csv_path);
    return false;
  }

  std::string line;
  bool header = true;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    if (header) {
      header = false;
      continue;
    }
    std::replace(line.begin(), line.end(), ',', '.');
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> cols;
    while (std::getline(ss, token, ';')) {
      cols.push_back(token);
    }
    if (cols.size() < 4) continue;

    try {
      double v_ms = std::stod(cols[1]);
      double kappa_max = std::stod(cols[2]);
      double kappa_rate_max = std::stod(cols[3]);
      lateral_limits_velocity_.push_back(v_ms);
      lateral_limits_kappa_max_.push_back(kappa_max);
      lateral_limits_kappa_rate_max_.push_back(kappa_rate_max);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "CSV parse error: " << ex.what());
      return false;
    }
  }

  if (lateral_limits_velocity_.empty() || lateral_limits_velocity_.size() != lateral_limits_kappa_max_.size() ||
      lateral_limits_velocity_.size() != lateral_limits_kappa_rate_max_.size()) {
    RCLCPP_ERROR_STREAM(get_logger(), "CSV contains no valid lateral limit data.");
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(), "Loaded lateral limits from CSV with " << lateral_limits_velocity_.size() << " entries.");
  return true;
}

void AckermannTrajectoryControl::UpdateLateralLimitsFromVelocity(const double velocity) {
  if (!use_speed_dependent_lateral_limits_ || !lateral_limits_loaded_) {
    max_curvature_current_ = max_curvature_;
    max_curvature_rate_current_ = max_curvature_rate_;
    return;
  }
  if (lateral_limits_velocity_.empty()) {
    max_curvature_current_ = max_curvature_;
    max_curvature_rate_current_ = max_curvature_rate_;
    return;
  }

  double v = velocity;
  if (v <= lateral_limits_velocity_.front()) {
    max_curvature_current_ = lateral_limits_kappa_max_.front();
    max_curvature_rate_current_ = lateral_limits_kappa_rate_max_.front();
    return;
  }
  if (v >= lateral_limits_velocity_.back()) {
    max_curvature_current_ = lateral_limits_kappa_max_.back();
    max_curvature_rate_current_ = lateral_limits_kappa_rate_max_.back();
    return;
  }
  if (!LinearInterpolation(lateral_limits_velocity_, lateral_limits_kappa_max_, v, max_curvature_current_)) {
    max_curvature_current_ = max_curvature_;
  }
  if (!LinearInterpolation(lateral_limits_velocity_, lateral_limits_kappa_rate_max_, v, max_curvature_rate_current_)) {
    max_curvature_rate_current_ = max_curvature_rate_;
  }
}

void AckermannTrajectoryControl::CalcOdometry(const double dt) {
  double yawRate = perception_msgs::object_access::getYawRate(cur_vehicle_state_);
  double velocity = perception_msgs::object_access::getVelLon(cur_vehicle_state_);
  odom_dy_ += sin(odom_dpsi_ + yawRate * 0.5 * dt) * velocity * dt;
  odom_dpsi_ += yawRate * dt;
}

void AckermannTrajectoryControl::ResetOdometry() {
  odom_dpsi_ = 0.0;
  odom_dy_ = 0.0;
}

bool AckermannTrajectoryControl::VehicleStateOk(const rclcpp::Time& ctrl_time) const {
  try {
    perception_msgs::object_access::sanityCheckContinuousState(cur_vehicle_state_);
  } catch (const std::exception&) {
    RCLCPP_ERROR_STREAM(get_logger(), "Sanity check for vehicle state failed!");
    return false;
  }
  double age = (ctrl_time - cur_vehicle_state_.header.stamp).seconds();
  if (age < 0.0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Vehicle state timestamp is newer than current control cycle time! Age: "
                                          << std::fixed << std::setprecision(15) << age << " seconds.");
    return false;
  } else if (age > vehicle_state_timeout_) {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Vehicle state is outdated! Age: " << std::fixed << std::setprecision(15) << age << " seconds.");
    return false;
  } else {
    return true;
  }
}

AckermannTrajectoryControl::SteeringCommand AckermannTrajectoryControl::UpdateKappaFromState(
    const perception_msgs::msg::EgoData& ego_data, const double wheelbase, double& kappa, double& kappa_rate) {
  SteeringCommand steering_command;
  steering_command.steering_angle = perception_msgs::object_access::getSteeringAngleAck(ego_data);
  steering_command.steering_angle_rate = perception_msgs::object_access::getSteeringAngleRateAck(ego_data);
  kappa = std::tan(steering_command.steering_angle) / wheelbase;
  double denom = wheelbase * std::cos(steering_command.steering_angle) * std::cos(steering_command.steering_angle);
  if (fabs(denom) > 1e-6) {
    kappa_rate = steering_command.steering_angle_rate / denom;
  } else {
    kappa_rate = 0.0;
  }
  return steering_command;
}

AckermannTrajectoryControl::LongitudinalCommand AckermannTrajectoryControl::UpdateLonFromState(
    const perception_msgs::msg::EgoData& ego_data) {
  LongitudinalCommand longitudinal_command;
  longitudinal_command.speed = perception_msgs::object_access::getVelLon(ego_data);
  longitudinal_command.acceleration = perception_msgs::object_access::getAccLon(ego_data);
  return longitudinal_command;
}

bool AckermannTrajectoryControl::LimitKappa(const double dt,
                                            double& kappa_tgt,
                                            double& kappa_rate,
                                            const double max_curvature,
                                            const double max_curvature_rate,
                                            const double max_curvature_accel,
                                            const double kappa_prev,
                                            const double kappa_rate_prev) {
  if (dt <= 0.0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Non-positive dt. Skipping curvature limiting...");
    return false;
  }

  bool kappa_limited = false;
  if (fabs(kappa_tgt) > max_curvature) {
    if (kappa_tgt >= 0.0) {
      kappa_tgt = max_curvature;
    } else {
      kappa_tgt = -max_curvature;
    }
    dy_pid_->ResetIntegral();
    RCLCPP_WARN_STREAM(get_logger(), "Curvature limited!");
    kappa_limited = true;
  }

  kappa_rate = 0.0;
  kappa_rate = (kappa_tgt - kappa_prev) / dt;

  if (fabs(kappa_rate) > max_curvature_rate && dt > 0.0) {
    if (kappa_rate >= 0.0) {
      kappa_rate = max_curvature_rate;
    } else {
      kappa_rate = -max_curvature_rate;
    }
    kappa_tgt = kappa_prev + kappa_rate * dt;
    dy_pid_->ResetIntegral();
    RCLCPP_WARN_STREAM(get_logger(), "Curvature-rate limited!");
    kappa_limited = true;
  }

  double kappa_accel = 0.0;
  kappa_accel = (kappa_rate - kappa_rate_prev) / dt;
  if (fabs(kappa_accel) > max_curvature_accel && dt > 0.0) {
    if (kappa_accel >= 0.0) {
      kappa_accel = max_curvature_accel;
    } else {
      kappa_accel = -max_curvature_accel;
    }
    kappa_rate = kappa_rate_prev + kappa_accel * dt;
    kappa_tgt = kappa_prev + kappa_rate * dt;
    dy_pid_->ResetIntegral();
    RCLCPP_WARN_STREAM(get_logger(), "Curvature-acceleration limited!");
    kappa_limited = true;
  }

  return kappa_limited;
}

AckermannTrajectoryControl::CurvatureCommand AckermannTrajectoryControl::LateralControl(const double dt) {
  const bool vehicle_standstill = perception_msgs::object_access::getStandstill(cur_vehicle_state_);
  if (vehicle_standstill) {
    // we reset the PID controllers in standstill to avoid integral windup and undesired overshoot when starting from standstill
    dy_pid_->ResetIntegral();
    dpsi_pid_->ResetIntegral();
  }

  // cascaded control
  double w_y = 0.0;
  double e_y = w_y - dy_;
  double w_psi = dy_pid_->Calc(e_y, dt);
  double e_psi = w_psi - dpsi_;
  double psi_dot_des = dpsi_pid_->Calc(e_psi, dt);

  double velocity = perception_msgs::object_access::getVelLon(cur_vehicle_state_);
  // be sure v!=0 (to avoid division by zero)
  if (fabs(velocity) < 0.1) {
    if (velocity < 0.0) {
      velocity = -0.1;
    } else {
      velocity = 0.1;
    }
  }

  double kappa_pid = std::tan(psi_dot_des * (wheelbase_ + self_st_gradient_ * velocity * velocity) / velocity) / wheelbase_;

  // ackermann feed-forward control (convert delta to kappa for feed-forward)
  double kappa_ff = std::tan(delta_tgt_) / wheelbase_;

  double kappa_tgt = kappa_pid + kappa_ff * feed_forward_gain_steering_angle_;

  double kappa_rate = 0.0;
  bool kappa_is_limited = LimitKappa(dt, kappa_tgt, kappa_rate, max_curvature_current_, max_curvature_rate_current_,
                                     max_curvature_accel_, last_kappa_, last_kappa_rate_);

  if (kappa_is_limited && !vehicle_standstill) {
    if (use_back_calculation_) {
      double kappa_fb_sat = kappa_tgt - kappa_ff * feed_forward_gain_steering_angle_;
      double denom = (wheelbase_ + self_st_gradient_ * velocity * velocity) / velocity;
      double psi_dot_sat = std::atan(kappa_fb_sat * wheelbase_) / denom;
      dpsi_pid_->BackCalculate(psi_dot_des, psi_dot_sat, dt, anti_windup_gain_);
    } else {
      dpsi_pid_->ResetIntegral();
    }
  }

  last_kappa_ = kappa_tgt;
  last_kappa_rate_ = kappa_rate;
  return CurvatureCommand{kappa_tgt, kappa_rate};
}

AckermannTrajectoryControl::LongitudinalCommand AckermannTrajectoryControl::LongitudinalControl(const double dt) {
  LongitudinalCommand longitudinal_command;
  double velocity = perception_msgs::object_access::getVelLon(cur_vehicle_state_);
  double w_v = v_tgt_;
  double e_v = w_v - velocity;
  double a_fb_v = dv_pid_->Calc(e_v, dt);

  double a_ff = a_tgt_ * feed_forward_gain_acceleration_;
  double a_ctrl = a_fb_v + a_ff;
  double a_unsat = a_ctrl;

  // limit desired acceleration
  if (a_ctrl > lon_max_acc_) {
    a_ctrl = lon_max_acc_;
    RCLCPP_WARN_STREAM(get_logger(), "Longitudinal acceleration limited!");
  } else if (a_ctrl < lon_min_acc_) {
    a_ctrl = lon_min_acc_;
    RCLCPP_WARN_STREAM(get_logger(), "Longitudinal acceleration limited!");
  }

  // calculate jerk with respect to last desired acceleration
  double jerk = 0.0;
  if (dt > 0.0) {
    jerk = (a_ctrl - vhcl_ctrl_output_.drive.acceleration) / dt;
  }
  if (fabs(jerk) > lon_max_jerk_ && dt > 0.0) {
    if (jerk >= 0.0) {
      jerk = lon_max_jerk_;
    } else {
      jerk = -lon_max_jerk_;
    }
    a_ctrl = vhcl_ctrl_output_.drive.acceleration + jerk * dt;
    RCLCPP_WARN_STREAM(get_logger(), "Longitudinal jerk limited!");
  }
  if (a_ctrl != a_unsat) {
    if (use_back_calculation_) {
      double a_fb_sat = a_ctrl - a_ff;
      dv_pid_->BackCalculate(a_fb_v, a_fb_sat, dt, anti_windup_gain_);
    } else {
      dv_pid_->ResetIntegral();
    }
  }

  longitudinal_command.speed = v_tgt_;
  longitudinal_command.acceleration = a_ctrl;
  longitudinal_command.jerk = jerk;
  return longitudinal_command;
}

/**
 * @brief Initializes ROS 2, starts the Ackermann trajectory controller node, and blocks until shutdown.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line argument values.
 * @return Exit status code reported to the operating system.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<AckermannTrajectoryControl>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(controller);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
