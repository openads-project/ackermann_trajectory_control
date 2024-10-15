/**
 * @file TrajectoryControlNode.cpp
 * @author Guido Küppers
 * @brief  ROS-Node for trajectory control.
 */

#include "TrajectoryControlNode.hpp"

#include <perception_msgs_utils/object_access.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

// ROS message parameters
const std::string TrajectoryControl::kInputTopicEgoData = "~/input_ego_data";
const std::string TrajectoryControl::kInputTopicTrajectory = "~/input_trajectory";
const std::string TrajectoryControl::kOutputTopic = "~/ctrl_cmds";


//Constructor of Trajectory Control Object
TrajectoryControl::TrajectoryControl() : Node("trajectory_controller") {
    loadParameters();
    setup();
}

TrajectoryControl::~TrajectoryControl() {
}

template <typename T>
void TrajectoryControl::declareAndLoadParameter(
    const std::string& name, T& member_param, const std::string& description,
    const bool add_to_auto_reconfigurable_params, const bool is_required, const bool read_only,
    const std::optional<T>& from_value, const std::optional<T>& to_value, const std::optional<T>& step_value,
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

void TrajectoryControl::loadParameters() {
    this->declareAndLoadParameter("vehicle_frame_id", vehicle_frame_id_, "Frame ID of the vehicle", false, false, false);
    this->declareAndLoadParameter("fixed_over_time_frame_id", fixed_over_time_frame_id_, "Frame ID of the fixed frame", false, false, false);
    this->declareAndLoadParameter("control_frequency", control_frequency_, "Control cycle frequency", true, false, false);//, 0.0, 200.0, 1.0); @jbusch why is this not working?
    this->declareAndLoadParameter("wheelbase", wheelbase_, "Wheelbase of the vehicle", false, false, false);
    this->declareAndLoadParameter("selfsteergradient", self_st_gradient_, "Self-steer-gradient of the vehicle", false, false, false);
    this->declareAndLoadParameter("longitudinal_lookahed_time", lon_t_lookahead_, "Longitudinal lookahead time", true, false, false);//, 0.0, 5.0, 0.1); @jbusch why is this not working?
    this->declareAndLoadParameter("lateral_lookahed_time", lat_t_lookahead_, "Lateral lookahead time", true, false, false);//, 0.0, 5.0, 0.1); @jbusch why is this not working?
    this->declareAndLoadParameter("max_longitudinal_acceleration", lon_max_acc_, "Maximum longitudinal acceleration", true, false, false);//, 0.0, 10.0, 0.1); @jbusch why is this not working?
    this->declareAndLoadParameter("min_longitudinal_acceleration", lon_min_acc_, "Minimum longitudinal acceleration", true, false, false);//, -10.0, 0.0, 0.1); @jbusch why is this not working?
    this->declareAndLoadParameter("max_longitudinal_jerk", lon_max_jerk_, "Maximum longitudinal jerk", true, false, false);//, 0.0, 20.0, 0.1); @jbusch why is this not working?
    this->declareAndLoadParameter("max_steering_angle", lat_max_st_ang_, "Maximum steering angle", false, false, false);//, 0.0, 90.0, 1.0); @jbusch why is this not working?
    lat_max_st_ang_*=M_PI/180.0;
    this->declareAndLoadParameter("max_steering_angle_rate", lat_max_st_rate_, "Maximum steering angle rate", false, false, false);//, 0.0, 270.0, 1.0); @jbusch why is this not working?
    lat_max_st_rate_*=M_PI/180.0;
    this->declareAndLoadParameter("velocity_lookup", gain_scheduling_velocity_lookup_, "Velocity lookup values", false, false, false);
    this->declareAndLoadParameter("feed_forward_acceleration_gain", vec_feed_forward_gain_acceleration_, "Feed forward acceleration gain", false, false, false);
    this->declareAndLoadParameter("feed_forward_steering_angle", vec_feed_forward_gain_steering_angle_, "Feed forward steering angle gain", false, false, false);
    this->declareAndLoadParameter("dv_p", dv_p_, "dv P Gain", false, false, false);
    this->declareAndLoadParameter("dv_i", dv_i_, "dv I Gain", false, false, false);
    this->declareAndLoadParameter("dv_d", dv_d_, "dv D Gain", false, false, false);
    this->declareAndLoadParameter("dy_p", dy_p_, "dy P Gain", false, false, false);
    this->declareAndLoadParameter("dy_i", dy_i_, "dy I Gain", false, false, false);
    this->declareAndLoadParameter("dy_d", dy_d_, "dy D Gain", false, false, false);
    this->declareAndLoadParameter("dpsi_p", dpsi_p_, "dpsi P Gain", false, false, false);
    this->declareAndLoadParameter("dpsi_i", dpsi_i_, "dpsi I Gain", false, false, false);
    this->declareAndLoadParameter("dpsi_d", dpsi_d_, "dpsi D Gain", false, false, false);
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult TrajectoryControl::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
      }
    }
    if(param.get_name() == "max_steering_angle") {
        lat_max_st_ang_ = param.get_value<double>()*M_PI/180.0;
    }
    if(param.get_name() == "max_steering_angle_rate") {
        lat_max_st_rate_ = param.get_value<double>()*M_PI/180.0;
    }
  }

  // mark parameter change successful
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

void TrajectoryControl::setup() {
    //Initialize dv-PID
    dv_pid_ = new PID(0.0, 0.0, 0.0);

    //Initialize dy-PID
    dy_pid_ = new PID(0.0, 0.0, 0.0);

    //Initialize dpsi-PID
    dpsi_pid_ = new PID(0.0, 0.0, 0.0);

    //Set Initial output values
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
    vehicle_state_sub_ = create_subscription<perception_msgs::msg::EgoData>(kInputTopicEgoData, 1, std::bind(&TrajectoryControl::VehicleStateCallback, this, std::placeholders::_1));
    trajectory_sub_ = create_subscription<trajectory_planning_msgs::msg::Trajectory>(kInputTopicTrajectory, 1, std::bind(&TrajectoryControl::TrajectoryCallback, this, std::placeholders::_1));

    // initialize publishers
    vehicle_ctrl_pub_ = create_publisher<ackermann_msgs::msg::AckermannDrive>(kOutputTopic,1);

    // initialize the cyclic vehicle-control timer; the callback VehicleCtrlCycle will be called wrt. the defined control frequency
    last_time_=now();
    vhcl_ctrl_timer_ = create_wall_timer(std::chrono::duration<double>(1.0/control_frequency_), std::bind(&TrajectoryControl::VehicleCtrlCycle, this));
}

//update the actual vehicle state
void TrajectoryControl::VehicleStateCallback(const perception_msgs::msg::EgoData::ConstPtr &msg)
{
    cur_vehicle_state_ = *msg;
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

//update the current trajectory
void TrajectoryControl::TrajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::ConstPtr &msg)
{
    subscribed_trajectory_ = *msg;
    ResetOdometry();
}

void TrajectoryControl::ResetController()
{
    a_tgt_=0.0;
    a_tgt_dv_=0.0;
    v_tgt_=0.0;
    y_tgt_=0.0;
    psi_tgt_=0.0;
    kappa_tgt_=0.0;
    dpsi_=0.0;
    dy_=0.0;
    dv_=0.0;
    dy_pid_->Reset();
    dpsi_pid_->Reset();
    dv_pid_->Reset();
    vhcl_ctrl_output_.drive.steering_angle = 0.0;
    vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
    vhcl_ctrl_output_.drive.speed = 0.0;
    vhcl_ctrl_output_.drive.acceleration = 0.0;
    vhcl_ctrl_output_.drive.jerk = 0.0;
    trajectory_planning_msgs::msg::Trajectory dummy_trj;
    subscribed_trajectory_=dummy_trj;
    tf_trajectory_=dummy_trj;
    perception_msgs::msg::EgoData dummy_state;
    cur_vehicle_state_=dummy_state;
    ResetOdometry();
}

void TrajectoryControl::setControllerGains()
{
    double velocity = perception_msgs::object_access::getVelLon(cur_vehicle_state_);
    // Feed-Forward Gains
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, vec_feed_forward_gain_acceleration_, velocity, feed_forward_gain_acceleration_)) feed_forward_gain_acceleration_=0.0;
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, vec_feed_forward_gain_steering_angle_, velocity, feed_forward_gain_steering_angle_)) feed_forward_gain_steering_angle_=0.0;
    
    double p, i, d;
    // dv Controller
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dv_p_, velocity, p)) p=0.0;
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dv_i_, velocity, i)) i=0.0;
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dv_d_, velocity, d)) d=0.0;
    dv_pid_->SetParameters(p, i, d);

    // dy Controller
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dy_p_, velocity, p)) p=0.0;
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dy_i_, velocity, i)) i=0.0;
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dy_d_, velocity, d)) d=0.0;
    dy_pid_->SetParameters(p, i, d);

    // dpsi Controller
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dpsi_p_, velocity, p)) p=0.0;
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dpsi_i_, velocity, i)) i=0.0;
    if(!LinearInterpolation(gain_scheduling_velocity_lookup_, dpsi_d_, velocity, d)) d=0.0;
    dpsi_pid_->SetParameters(p, i, d);
}


//perform the vehicle control cycle
void TrajectoryControl::VehicleCtrlCycle()
{
    if(last_time_ > now())
    {
        RCLCPP_WARN_STREAM(get_logger(), "Resetting controller because of Jump-Back in time!");
        ResetController();
    }
    last_time_=now();
    if (!InputSanityCheck()) //some inputs are not ok
    {
        //don't do anything
        return;
    }

    // Standstill signal?
    if(tf_trajectory_.standstill)
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "Standstill.");
        vhcl_ctrl_output_.drive.steering_angle = 0.0;
        vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
        vhcl_ctrl_output_.drive.speed = 0.0;
        vhcl_ctrl_output_.drive.acceleration = 0.0;
        vhcl_ctrl_output_.drive.jerk = 0.0;
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        dv_pid_->Reset();
    }
    else
    {
        if(!TrjDataProc())
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Processing of input Trajectory failed!");
            return;
        }
        setControllerGains();
        vhcl_ctrl_output_.drive.steering_angle = LateralControl();
        vhcl_ctrl_output_.drive.acceleration = LongitudinalControl();
        if(std::isnan(vhcl_ctrl_output_.drive.steering_angle))
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Steering Angle Output Value isNaN!");
            vhcl_ctrl_output_.drive.steering_angle=0.0;
            dy_pid_->Reset();
            dpsi_pid_->Reset();
            return;
        }
        if(std::isnan(vhcl_ctrl_output_.drive.acceleration))
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Target Acceleration Output Value isNaN!");
            vhcl_ctrl_output_.drive.acceleration=0.0;
            dv_pid_->Reset();
            return;
        }
    }
    vhcl_ctrl_output_.header.stamp = now();
    // Publish unstamped message
    vehicle_ctrl_pub_->publish(vhcl_ctrl_output_.drive);
}

bool TrajectoryControl::InputSanityCheck()
{
    double age;
    age = (now() - cur_vehicle_state_.header.stamp).seconds();
    if (age > 0.2 || age < 0.0) //current vehicle state data older than 0.2s
    {
        RCLCPP_ERROR_STREAM(get_logger(), "EgoState-Data outdated!");
        return false;
    }
    if (trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_) == 0)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Input Trajctory is empty!");
        return false;
    } else {
      // get last state of trajectory
      double last_time = trajectory_planning_msgs::trajectory_access::getT(tf_trajectory_, trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_) - 1);
      double lookahead = std::max(lon_t_lookahead_, lat_t_lookahead_);
      if (last_time < 2 * lookahead) {
        RCLCPP_ERROR_STREAM(get_logger(), "Trajectory is too short!");
        return false;
      }
    }

    return true;
}

bool TrajectoryControl::TrjDataProc()
{
    // Derive State Vectors
    std::vector<double> TIME, V, A, Y, THETA, KAPPA;
    int n_samples = trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory_);
    for(int i=0; i<n_samples; i++){
        TIME.push_back(trajectory_planning_msgs::trajectory_access::getT(tf_trajectory_, i));
        V.push_back(trajectory_planning_msgs::trajectory_access::getV(tf_trajectory_, i));
        Y.push_back(trajectory_planning_msgs::trajectory_access::getY(tf_trajectory_, i));
        if(tf_trajectory_.type_id==trajectory_planning_msgs::DRIVABLE::TYPE_ID)
        {
            A.push_back(trajectory_planning_msgs::trajectory_access::getA(tf_trajectory_, i));
            THETA.push_back(trajectory_planning_msgs::trajectory_access::getTheta(tf_trajectory_, i));
            KAPPA.push_back(trajectory_planning_msgs::trajectory_access::getKappa(tf_trajectory_, i));
        }
        else
        {
            // To-Do Fill A, THETA and KAPPA with finite-differences
            RCLCPP_ERROR_STREAM(get_logger(), "trajectory_planning_msgs::DRIVABLE-Type is currently not supported!");
            return false;
        }
    }

    //calculate desired interpolation time for longitudinal values
    double des_time = (now() - tf_trajectory_.header.stamp).seconds() + lon_t_lookahead_;
    //interpolate longitudinal target values
    if(!LinearInterpolation(TIME, V, des_time, v_tgt_)) return false;
    if(!LinearInterpolation(TIME, A, des_time, a_tgt_)) return false;

    //switch to desired interpolation time for lateral values
    des_time = des_time - lon_t_lookahead_ + lat_t_lookahead_;
    //interpolate lateral target values
    if(!LinearInterpolation(TIME, Y, des_time, y_tgt_)) return false;
    if(!LinearInterpolation(TIME, THETA, des_time, psi_tgt_)) return false;
    if(!LinearInterpolation(TIME, KAPPA, des_time, kappa_tgt_)) return false;

    //CalcOdometry
    double dt = (now() - vhcl_ctrl_output_.header.stamp).seconds();
    CalcOdometry(dt); //Cyclic Control

    dy_ = odom_dy_ - y_tgt_;
    dpsi_ = odom_dpsi_ - psi_tgt_;
    return true;
}

bool TrajectoryControl::LinearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y)
{
    if (desired_x < *min_element(X.begin(), X.end()) || desired_x > *max_element(X.begin(), X.end()))
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Desired X-Value is not in between of X-Min and X-Max of the given vector!");
        return false;
    }
    if(X.size() != Y.size())
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Input vectors don't have the same length!");
        return false;
    }

    //go through array and search for sampling points
    int i;
    for(i = 0; i < X.size(); i++)
    {
        if (X[i] < desired_x)
        {
            continue;
        }
        else if (X[i] == desired_x)
        {
            output_y = Y[i];
            return true;
        }
        else
        {
            break;
        }
    }
    output_y = Y[i - 1] + ((Y[i] - Y[i - 1]) / (X[i] - X[i - 1])) * (desired_x - X[i - 1]);
    return true;
}

void TrajectoryControl::CalcOdometry(double dt)
{
    double yawRate = perception_msgs::object_access::getYawRate(cur_vehicle_state_);
    double velocity = perception_msgs::object_access::getVelLon(cur_vehicle_state_);
    odom_dy_ += sin(odom_dpsi_ + yawRate * 0.5 * dt) * velocity * dt;
    odom_dpsi_ += yawRate * dt;
}

void TrajectoryControl::ResetOdometry()
{
    odom_dpsi_ = 0.0;
    odom_dy_ = 0.0;
}

double TrajectoryControl::LateralControl()
{
    //cascaded control
    double dt = (now() - vhcl_ctrl_output_.header.stamp).seconds();
    double w_y = 0.0;
    double e_y = w_y - dy_;
    double w_psi = dy_pid_->Calc(e_y, dt);
    double e_psi = w_psi - dpsi_;
    double psi_dot_des = dpsi_pid_->Calc(e_psi, dt);

    double velocity = perception_msgs::object_access::getVelLon(cur_vehicle_state_);
    //be sure v!=0 (to avoid division by zero)
    if (fabs(velocity) < 0.1)
    {
        if (velocity < 0.0)
        {
            velocity = -0.1;
        }
        else
        {
            velocity = 0.1;
        }
    }

    double st_ang_pid = psi_dot_des * (wheelbase_ + self_st_gradient_ * velocity * velocity) / velocity;

    //ackermann feed-forward control with trajectory kappa
    double st_ang_ack = atan(wheelbase_ * kappa_tgt_);

    double st_ang = st_ang_pid + st_ang_ack * feed_forward_gain_steering_angle_;

    if (perception_msgs::object_access::getStandstill(cur_vehicle_state_)) //Standstill-Situation
    {
        dy_pid_->Reset();
        dpsi_pid_->Reset();
    }

    //Limit desired steering angle
    if (fabs(st_ang) > lat_max_st_ang_)
    {
        if (st_ang >= 0)
        {
            st_ang = lat_max_st_ang_;
        }
        else
        {
            st_ang = -lat_max_st_ang_;
        }
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        RCLCPP_DEBUG_STREAM(get_logger(), "Steering-Angle limited!");
    }
    //calculate steering rate with respect to last steering angle
    double st_rate = (st_ang - vhcl_ctrl_output_.drive.steering_angle) / dt;
    if (fabs(st_rate) > lat_max_st_rate_)
    {
        if (st_rate >= 0)
        {
            st_ang = vhcl_ctrl_output_.drive.steering_angle + lat_max_st_rate_ * dt;
        }
        else
        {
            st_ang = vhcl_ctrl_output_.drive.steering_angle - lat_max_st_rate_ * dt;
        }
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        RCLCPP_DEBUG_STREAM(get_logger(), "Steering-rate limited!");
    }

    return st_ang;
}

double TrajectoryControl::LongitudinalControl()
{
    double dt = (now() - vhcl_ctrl_output_.header.stamp).seconds();
    double velocity = perception_msgs::object_access::getVelLon(cur_vehicle_state_);
    double w_v = v_tgt_;
    double e_v = w_v - velocity;
    double a_fb_v = dv_pid_->Calc(e_v, dt);

    double a_ctrl = a_fb_v + a_tgt_ * feed_forward_gain_acceleration_;

    //Limit desired acceleration
    if (a_ctrl > lon_max_acc_)
    {
        a_ctrl = lon_max_acc_;
        dv_pid_->Reset();
        RCLCPP_DEBUG_STREAM(get_logger(), "Longitudinal acceleration limited!");
    }
    else if (a_ctrl < lon_min_acc_)
    {
        a_ctrl = lon_min_acc_;
        dv_pid_->Reset();
        RCLCPP_DEBUG_STREAM(get_logger(), "Longitudinal acceleration limited!");
    }

    //calculate jerk with respect to last desired acceleration
    double jerk = (a_ctrl - vhcl_ctrl_output_.drive.acceleration) / dt;
    if (fabs(jerk) > lon_max_jerk_)
    {
        if (jerk >= 0.0)
        {
            a_ctrl = vhcl_ctrl_output_.drive.acceleration + lon_max_jerk_ * dt;
        }
        else
        {
            a_ctrl = vhcl_ctrl_output_.drive.acceleration - lon_max_jerk_ * dt;
        }
        //dv_pid_->Reset();
        RCLCPP_DEBUG_STREAM(get_logger(), "Longitudinal jerk limited!");
    }
    vhcl_ctrl_output_.drive.speed = v_tgt_;
    return a_ctrl;
}

// Main of Trajectory Control Node
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<TrajectoryControl>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}