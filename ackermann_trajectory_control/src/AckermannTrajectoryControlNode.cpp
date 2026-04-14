// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include "AckermannTrajectoryControlNode.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

AckermannTrajectoryControl::AckermannTrajectoryControl() : Node("ackermann_trajectory_controller") {
  // declare and load node parameters
  this->declareAndLoadParameter("vehicle_frame_id", vehicle_frame_id_, "Frame ID of the vehicle", false);
  this->declareAndLoadParameter("fixed_over_time_frame_id", fixed_over_time_frame_id_,
                                "Frame ID of the fixed frame used for transformations over time (e.g. map)", false);
  this->declareAndLoadParameter("control_frequency", control_frequency_, "Frequency of the control loop in Hz", true, false,
                                false, std::optional<double>{0.0}, std::optional<double>{200.0}, std::optional<double>{1.0});
  this->declareAndLoadParameter("vehicle_state_timeout", vehicle_state_timeout_, "Maximum allowed age of the ego data in seconds",
                                true, false, false, std::optional<double>{0.0}, std::optional<double>{1.0},
                                std::optional<double>{0.05});
  this->declareAndLoadParameter("wheelbase", wheelbase_, "Wheelbase of the vehicle in meters (required for lateral control)",
                                false);
  this->declareAndLoadParameter("selfsteergradient", self_st_gradient_,
                                "Self-steer gradient of the vehicle (required for lateral control)", false);
  this->declareAndLoadParameter("longitudinal_lookahead_time", lon_t_lookahead_,
                                "Time in seconds for the longitudinal look-ahead", true, false, false, std::optional<double>{0.0},
                                std::optional<double>{5.0}, std::optional<double>{0.1});
  this->declareAndLoadParameter("lateral_lookahead_time", lat_t_lookahead_, "Time in seconds for the lateral look-ahead", true,
                                false, false, std::optional<double>{0.0}, std::optional<double>{5.0}, std::optional<double>{0.1});
  this->declareAndLoadParameter("max_longitudinal_acceleration", lon_max_acc_,
                                "Maximum allowed longitudinal acceleration in m/s^2 (constraint)", true, false, false,
                                std::optional<double>{0.0}, std::optional<double>{10.0}, std::optional<double>{0.1});
  this->declareAndLoadParameter("min_longitudinal_acceleration", lon_min_acc_,
                                "Minimum allowed longitudinal acceleration in m/s^2 (constraint, should be negative)", true,
                                false, false, std::optional<double>{-10.0}, std::optional<double>{0.0},
                                std::optional<double>{0.1});
  this->declareAndLoadParameter("max_longitudinal_jerk", lon_max_jerk_,
                                "Maximum allowed longitudinal jerk in m/s^3 (constraint, absolute value)", true, false, false,
                                std::optional<double>{0.0}, std::optional<double>{20.0}, std::optional<double>{0.1});
  this->declareAndLoadParameter("max_curvature", max_curvature_, "Maximum allowed curvature (constraint, absolute value)", false,
                                false, false, std::optional<double>{0.0}, std::optional<double>{1.0},
                                std::optional<double>{1e-12});
  this->declareAndLoadParameter("max_curvature_rate", max_curvature_rate_,
                                "Maximum allowed curvature rate (constraint, absolute value)", false, false, false,
                                std::optional<double>{0.0}, std::optional<double>{5.0}, std::optional<double>{1e-12});
  this->declareAndLoadParameter("max_curvature_acceleration", max_curvature_accel_,
                                "Maximum allowed curvature acceleration (constraint, absolute value)", false, false, false,
                                std::optional<double>{0.0}, std::optional<double>{20.0}, std::optional<double>{1e-12});
  this->declareAndLoadParameter("use_speed_dependent_lateral_limits", use_speed_dependent_lateral_limits_,
                                "Boolean indicating whether the controller uses speed-dependent curvature limits from a CSV file",
                                false);
  this->declareAndLoadParameter("lateral_limits_csv", lateral_limits_csv_path_,
                                "CSV file path for speed-dependent curvature limits", false);
  this->declareAndLoadParameter("anti_windup_gain", anti_windup_gain_, "Anti-windup back-calculation gain", true, false, false,
                                std::optional<double>{0.0}, std::optional<double>{100.0}, std::optional<double>{0.1});
  this->declareAndLoadParameter("use_back_calculation", use_back_calculation_, "Enable anti-windup back-calculation", true);
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

  max_curvature_current_ = max_curvature_;
  max_curvature_rate_current_ = max_curvature_rate_;
  this->setup();
}

AckermannTrajectoryControl::~AckermannTrajectoryControl() {}

template <typename T>
void AckermannTrajectoryControl::declareAndLoadParameter(const std::string& name,
                                                         T& member_param,
                                                         const std::string& description,
                                                         const bool add_to_auto_reconfigurable_params,
                                                         const bool is_required,
                                                         const bool read_only,
                                                         const std::optional<T>& from_value,
                                                         const std::optional<T>& to_value,
                                                         const std::optional<T>& step_value,
                                                         const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto param_type = rclcpp::ParameterValue(member_param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = step_value.has_value() ? step_value.value() : 0;
      range.set__from_value(from_value.value()).set__to_value(to_value.value()).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = step_value.has_value() ? step_value.value() : 0.0;
      range.set__from_value(from_value.value()).set__to_value(to_value.value()).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type does not support range.");
    }
  }

  this->declare_parameter(name, param_type, param_desc);

  try {
    member_param = this->get_parameter(name).get_value<T>();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Parameter '" << name << "' not set but required. Exiting.");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Parameter '" << name << "' not set. Using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : member_param) ss << element << (&element != &member_param.back() ? ", " : "]");
      } else {
        ss << member_param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&member_param](const rclcpp::Parameter& param) {
      member_param = param.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

rcl_interfaces::msg::SetParametersResult AckermannTrajectoryControl::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
      }
    }
    if (param.get_name() == "max_curvature") {
      max_curvature_ = param.get_value<double>();
      if (!use_speed_dependent_lateral_limits_) {
        max_curvature_current_ = max_curvature_;
      }
    }
    if (param.get_name() == "max_curvature_rate") {
      max_curvature_rate_ = param.get_value<double>();
      if (!use_speed_dependent_lateral_limits_) {
        max_curvature_rate_current_ = max_curvature_rate_;
      }
    }
    if (param.get_name() == "max_curvature_acceleration") {
      max_curvature_accel_ = param.get_value<double>();
    }
    if (param.get_name() == "use_speed_dependent_lateral_limits") {
      use_speed_dependent_lateral_limits_ = param.get_value<bool>();
    }
    if (param.get_name() == "lateral_limits_csv") {
      lateral_limits_csv_path_ = param.get_value<std::string>();
    }
    if (param.get_name() == "use_back_calculation") {
      use_back_calculation_ = param.get_value<bool>();
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

  ResetOdometry();

  // initialize tf listener
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

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

  // initialize the cyclic vehicle-control timer; the callback VehicleCtrlCycle will be called wrt. the defined control frequency
  last_time_ = now();
  vhcl_ctrl_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_frequency_),
                                       std::bind(&AckermannTrajectoryControl::VehicleCtrlCycle, this));

  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&AckermannTrajectoryControl::parametersCallback, this, std::placeholders::_1));

  // Annotate message links for tracing: Publish ackermann commands periodically, depending an all subscribed topics.
  std::vector<const void*> link_subs;
  link_subs.push_back(static_cast<const void*>(vehicle_state_sub_->get_subscription_handle().get()));
  link_subs.push_back(static_cast<const void*>(trajectory_sub_->get_subscription_handle().get()));
  std::vector<const void*> link_pubs;
  link_pubs.push_back(static_cast<const void*>(vehicle_ctrl_pub_->get_publisher_handle().get()));
  TRACETOOLS_TRACEPOINT(message_link_periodic_async, link_subs.data(), link_subs.size(), link_pubs.data(), link_pubs.size());
}

void AckermannTrajectoryControl::VehicleStateCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  cur_vehicle_state_ = *msg;
  UpdateLateralLimitsFromVelocity(perception_msgs::object_access::getVelLon(cur_vehicle_state_));
  // transform latest trajectory to current vehicle-frame
  trajectory_planning_msgs::msg::Trajectory tf_trajectory;
  try {
    tf_trajectory_ =
        tf2_buffer_->transform(subscribed_trajectory_, vehicle_frame_id_, tf2_ros::fromMsg(cur_vehicle_state_.header.stamp),
                               fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation is not available. Ex: %s", ex.what());
    tf_trajectory_ = subscribed_trajectory_;
  }
  ResetOdometry();
}

void AckermannTrajectoryControl::TrajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  subscribed_trajectory_ = *msg;
  // check needs to be performed before any transformation because x=0, y=0, theta=0 is indicating a high-level-initialization
  if (trajectory_planning_msgs::trajectory_access::getSamplePointSize(subscribed_trajectory_) > 0) {
    // get x, y and theta of trajectory at first state
    double x = trajectory_planning_msgs::trajectory_access::getX(subscribed_trajectory_, 0);
    double y = trajectory_planning_msgs::trajectory_access::getY(subscribed_trajectory_, 0);
    double theta = trajectory_planning_msgs::trajectory_access::getTheta(subscribed_trajectory_, 0);
    if (x == 0.0 && y == 0.0 && theta == 0.0) {  // high-level-initialization
      dy_pid_->Reset();
      dpsi_pid_->Reset();
      dv_pid_->Reset();
    }
  }

  // transform latest trajectory to current vehicle-frame
  trajectory_planning_msgs::msg::Trajectory tf_trajectory;
  try {
    tf_trajectory_ =
        tf2_buffer_->transform(subscribed_trajectory_, vehicle_frame_id_, tf2_ros::fromMsg(cur_vehicle_state_.header.stamp),
                               fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation is not available. Ex: %s", ex.what());
    tf_trajectory_ = subscribed_trajectory_;
  }
}

void AckermannTrajectoryControl::LatActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) { lat_active_ = msg->data; }

void AckermannTrajectoryControl::LonActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) { lon_active_ = msg->data; }

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
  trajectory_planning_msgs::msg::Trajectory dummy_trj;
  subscribed_trajectory_ = dummy_trj;
  tf_trajectory_ = dummy_trj;
  perception_msgs::msg::EgoData dummy_state;
  cur_vehicle_state_ = dummy_state;
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
  if (last_time_ > now()) {
    RCLCPP_WARN_STREAM(get_logger(), "Resetting controller because of Jump-Back in time!");
    ResetController();
  }
  last_time_ = now();
  bool vehicle_state_ok = VehicleStateOk();
  if (!lat_active_ && vehicle_state_ok) {
    UpdateKappaFromState();
  }
  if (!lon_active_ && vehicle_state_ok) {
    UpdateLonFromState();
  }
  if (!lat_active_) {
    dy_pid_->Reset();
    dpsi_pid_->Reset();
  }
  if (!lon_active_) {
    dv_pid_->Reset();
  }
  if (!InputSanityCheck())  // some inputs are not ok
  {
    // don't do anything
    return;
  }

  // Hold zero output while the trajectory explicitly requests standstill.
  if (tf_trajectory_.standstill) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Standstill.");
    vhcl_ctrl_output_.drive.steering_angle = 0.0;
    vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
    vhcl_ctrl_output_.drive.speed = 0.0;
    vhcl_ctrl_output_.drive.acceleration = 0.0;
    vhcl_ctrl_output_.drive.jerk = 0.0;
    dy_pid_->Reset();
    dpsi_pid_->Reset();
    dv_pid_->Reset();
    last_kappa_ = 0.0;
    last_kappa_rate_ = 0.0;
  } else {
    if (!TrjDataProc()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Processing of input Trajectory failed!");
      return;
    }
    setControllerGains();
    double dt = (now() - vhcl_ctrl_output_.header.stamp).seconds();
    if (dt <= 0.0) return;
    if (lat_active_) {
      double kappa_tgt = LateralControl(dt);
      double st_ang = std::atan(kappa_tgt * wheelbase_);
      if (std::isnan(st_ang)) {
        RCLCPP_ERROR_STREAM(get_logger(), "Steering Angle Output Value isNaN!");
        vhcl_ctrl_output_.drive.steering_angle = 0.0;
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        return;
      }
      vhcl_ctrl_output_.drive.steering_angle = static_cast<float>(st_ang);
    } else {
      double st_ang = UpdateKappaFromState();
      vhcl_ctrl_output_.drive.steering_angle = static_cast<float>(st_ang);
    }
    if (lon_active_) {
      vhcl_ctrl_output_.drive.acceleration = static_cast<float>(LongitudinalControl(dt));
      if (std::isnan(vhcl_ctrl_output_.drive.acceleration)) {
        RCLCPP_ERROR_STREAM(get_logger(), "Target Acceleration Output Value isNaN!");
        vhcl_ctrl_output_.drive.acceleration = 0.0;
        dv_pid_->Reset();
        return;
      }
    } else {
      UpdateLonFromState();
    }
  }
  vhcl_ctrl_output_.header.stamp = now();
  vehicle_ctrl_pub_->publish(vhcl_ctrl_output_);
}

bool AckermannTrajectoryControl::InputSanityCheck() {
  if (!VehicleStateOk()) {
    RCLCPP_DEBUG_STREAM(get_logger(), "EgoState-Data outdated!");
    return false;
  }
  if (trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_) == 0) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Input Trajctory is empty!");
    return false;
  } else {
    // get last state of trajectory
    double last_time = trajectory_planning_msgs::trajectory_access::getT(
        tf_trajectory_, trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_) - 1);
    double lookahead = std::max(lon_t_lookahead_, lat_t_lookahead_);
    if (last_time < 2 * lookahead) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Trajectory is too short!");
      return false;
    }
  }
  return true;
}

bool AckermannTrajectoryControl::TrjDataProc() {
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
  double delta_time = (now() - tf_trajectory_.header.stamp).seconds();
  // interpolate longitudinal target values
  if (!LinearInterpolation(TIME, V, delta_time + lon_t_lookahead_, v_tgt_)) return false;
  if (!LinearInterpolation(TIME, A, delta_time + lon_t_lookahead_, a_tgt_)) return false;

  // interpolate lateral target values
  if (!LinearInterpolation(TIME, Y, delta_time + lat_t_lookahead_, y_tgt_)) return false;
  if (!LinearInterpolation(TIME, THETA, delta_time + lat_t_lookahead_, psi_tgt_)) return false;
  if (!LinearInterpolation(TIME, DELTA, delta_time + lat_t_lookahead_, delta_tgt_)) return false;

  // CalcOdometry
  auto now = this->now();
  double dt_ego = (now - cur_vehicle_state_.header.stamp).seconds();
  double dt_ctrl = (now - vhcl_ctrl_output_.header.stamp).seconds();
  double dt = std::min(dt_ego, dt_ctrl);
  CalcOdometry(dt);  // Cyclic Control

  dy_ = odom_dy_ - y_tgt_;
  dpsi_ = odom_dpsi_ - psi_tgt_;
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

bool AckermannTrajectoryControl::VehicleStateOk() const {
  double age = (now() - cur_vehicle_state_.header.stamp).seconds();
  return age <= vehicle_state_timeout_ && age >= 0.0;
}

double AckermannTrajectoryControl::UpdateKappaFromState() {
  double st_ang = perception_msgs::object_access::getSteeringAngleAck(cur_vehicle_state_);
  double st_rate = perception_msgs::object_access::getSteeringAngleRateAck(cur_vehicle_state_);
  last_kappa_ = std::tan(st_ang) / wheelbase_;
  double denom = wheelbase_ * std::cos(st_ang) * std::cos(st_ang);
  if (fabs(denom) > 1e-6) {
    last_kappa_rate_ = st_rate / denom;
  } else {
    last_kappa_rate_ = 0.0;
  }
  return st_ang;
}

void AckermannTrajectoryControl::UpdateLonFromState() {
  vhcl_ctrl_output_.drive.acceleration = static_cast<float>(perception_msgs::object_access::getAccLon(cur_vehicle_state_));
  vhcl_ctrl_output_.drive.speed = static_cast<float>(perception_msgs::object_access::getVelLon(cur_vehicle_state_));
}

double AckermannTrajectoryControl::LateralControl(const double dt) {
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

  if (perception_msgs::object_access::getStandstill(cur_vehicle_state_))  //Standstill-Situation
  {
    dy_pid_->Reset();
    dpsi_pid_->Reset();
  }

  bool kappa_limited = false;
  if (fabs(kappa_tgt) > max_curvature_current_) {
    if (kappa_tgt >= 0.0) {
      kappa_tgt = max_curvature_current_;
    } else {
      kappa_tgt = -max_curvature_current_;
    }
    dy_pid_->ResetIntegral();
    RCLCPP_WARN_STREAM(get_logger(), "Curvature limited!");
    kappa_limited = true;
  }

  double kappa_prev = last_kappa_;
  double kappa_rate = 0.0;
  if (dt > 0.0) {
    kappa_rate = (kappa_tgt - kappa_prev) / dt;
  }
  if (fabs(kappa_rate) > max_curvature_rate_current_ && dt > 0.0) {
    if (kappa_rate >= 0.0) {
      kappa_rate = max_curvature_rate_current_;
    } else {
      kappa_rate = -max_curvature_rate_current_;
    }
    kappa_tgt = kappa_prev + kappa_rate * dt;
    dy_pid_->ResetIntegral();
    RCLCPP_WARN_STREAM(get_logger(), "Curvature-rate limited!");
    kappa_limited = true;
  }

  double kappa_accel = 0.0;
  if (dt > 0.0) {
    kappa_accel = (kappa_rate - last_kappa_rate_) / dt;
  }
  if (fabs(kappa_accel) > max_curvature_accel_ && dt > 0.0) {
    if (kappa_accel >= 0.0) {
      kappa_accel = max_curvature_accel_;
    } else {
      kappa_accel = -max_curvature_accel_;
    }
    kappa_rate = last_kappa_rate_ + kappa_accel * dt;
    kappa_tgt = kappa_prev + kappa_rate * dt;
    dy_pid_->ResetIntegral();
    RCLCPP_WARN_STREAM(get_logger(), "Curvature-acceleration limited!");
    kappa_limited = true;
  }

  if (kappa_limited) {
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

  return kappa_tgt;
}

double AckermannTrajectoryControl::LongitudinalControl(const double dt) {
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
  double jerk = (a_ctrl - vhcl_ctrl_output_.drive.acceleration) / dt;
  if (fabs(jerk) > lon_max_jerk_ && dt > 0.0) {
    if (jerk >= 0.0) {
      a_ctrl = vhcl_ctrl_output_.drive.acceleration + lon_max_jerk_ * dt;
    } else {
      a_ctrl = vhcl_ctrl_output_.drive.acceleration - lon_max_jerk_ * dt;
    }
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
  vhcl_ctrl_output_.drive.speed = static_cast<float>(v_tgt_);
  return a_ctrl;
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
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
