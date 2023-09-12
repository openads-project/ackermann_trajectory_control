/**
 * @file TrajectoryControlNode.cpp
 * @author Guido Küppers
 * @brief  ROS-Node for trajectory control.
 */

#include "TrajectoryControlNode.hpp"

#include <perception_interfaces/object_access.hpp>
#include <trajectory_interfaces/trajectory_access.hpp>


//Main of Trajectory Control Node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<TrajectoryControl>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}

//Constructor of Trajectory Control Object
TrajectoryControl::TrajectoryControl() : Node("trajectory_controller")
{

    vehicle_state_sub_ = create_subscription<perception_interfaces::msg::EgoData>("/carla_its_adapter/ego_data", 1, std::bind(&TrajectoryControl::VehicleStateCallback, this, std::placeholders::_1));
    trajectory_sub_ = create_subscription<trajectory_interfaces::msg::Trajectory>("/trajectory_supervision_node/output_topic", 1, std::bind(&TrajectoryControl::TrajectoryCallback, this, std::placeholders::_1));

    vehicle_ctrl_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("~/ctrl_cmds",1);

    loadParameters();

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

    //Initialization of the cyclic vehicle-control timer; the callback VehicleCtrlCycle will be called every 0.01s e.g. with 100Hz.
    last_time_=now();
    vhcl_ctrl_timer_ = create_wall_timer(std::chrono::duration<double>(1.0/control_frequency_), std::bind(&TrajectoryControl::VehicleCtrlCycle, this));
}

TrajectoryControl::~TrajectoryControl()
{
}

void TrajectoryControl::loadParameters() {

    // General parameters
    // Control cycle frequency
    declare_parameter("control_frequency", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        control_frequency_ = get_parameter("control_frequency").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'control_frequency\' set to: " << control_frequency_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'control_frequency\' is not set correctly, using default value: " << control_frequency_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'control_frequency\' is not set, using default value: " << control_frequency_);
    }

    // Constant vehicle parameters
    // Wheelbase
    declare_parameter("wheelbase", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        wheelbase_ = get_parameter("wheelbase").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'wheelbase\' set to: " << wheelbase_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'wheelbase\' is not set correctly, using default value: " << wheelbase_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'wheelbase\' is not set, using default value: " << wheelbase_);
    }

    // Self-Steer-Gradient
    declare_parameter("selfsteergradient", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        self_st_gradient_ = get_parameter("selfsteergradient").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'selfsteergradient\' set to: " << self_st_gradient_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'selfsteergradient\' is not set correctly, using default value: " << self_st_gradient_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'selfsteergradient\' is not set, using default value: " << self_st_gradient_);
    }

    // Longitudinal Lookahead Time
    declare_parameter("longitudinal_lookahed_time", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        lon_t_lookahead_ = get_parameter("longitudinal_lookahed_time").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'longitudinal_lookahed_time\' set to: " << lon_t_lookahead_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'longitudinal_lookahed_time\' is not set correctly, using default value: " << lon_t_lookahead_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'longitudinal_lookahed_time\' is not set, using default value: " << lon_t_lookahead_);
    }

    // Lateral Lookahead Time
    declare_parameter("lateral_lookahed_time", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        lat_t_lookahead_ = get_parameter("lateral_lookahed_time").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'lateral_lookahed_time\' set to: " << lat_t_lookahead_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'lateral_lookahed_time\' is not set correctly, using default value: " << lat_t_lookahead_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'lateral_lookahed_time\' is not set, using default value: " << lat_t_lookahead_);
    }

    // Maximum Longitudinal Acceleration
    declare_parameter("max_longitudinal_acceleration", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        lon_max_acc_ = get_parameter("max_longitudinal_acceleration").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'max_longitudinal_acceleration\' set to: " << lon_max_acc_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'max_longitudinal_acceleration\' is not set correctly, using default value: " << lon_max_acc_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'max_longitudinal_acceleration\' is not set, using default value: " << lon_max_acc_);
    }

    // Minimum Longitudinal Acceleration
    declare_parameter("min_longitudinal_acceleration", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        lon_min_acc_ = get_parameter("min_longitudinal_acceleration").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'min_longitudinal_acceleration\' set to: " << lon_min_acc_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'min_longitudinal_acceleration\' is not set correctly, using default value: " << lon_min_acc_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'min_longitudinal_acceleration\' is not set, using default value: " << lon_min_acc_);
    }

    // Maximum Longitudinal Jerk
    declare_parameter("max_longitudinal_jerk", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        lon_max_jerk_ = get_parameter("max_longitudinal_jerk").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'max_longitudinal_jerk\' set to: " << lon_max_jerk_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'max_longitudinal_jerk\' is not set correctly, using default value: " << lon_max_jerk_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'max_longitudinal_jerk\' is not set, using default value: " << lon_max_jerk_);
    }

    // Maximum Steering Angle
    declare_parameter("max_steering_angle", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        lat_max_st_ang_ = get_parameter("max_steering_angle").as_double()*M_PI/180.0;
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'max_steering_angle\' set to: " << lat_max_st_ang_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'max_steering_angle\' is not set correctly, using default value: " << lat_max_st_ang_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'max_steering_angle\' is not set, using default value: " << lat_max_st_ang_);
    }

    // Maximum Steering Angle Rate
    declare_parameter("max_steering_angle_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        lat_max_st_rate_ = get_parameter("max_steering_angle").as_double()*M_PI/180.0;
        RCLCPP_INFO_STREAM(get_logger(), "Parameter \'max_steering_angle\' set to: " << lat_max_st_rate_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'max_steering_angle\' is not set correctly, using default value: " << lat_max_st_rate_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter \'max_steering_angle\' is not set, using default value: " << lat_max_st_rate_);
    }

    // Velocity Lookup Values
    declare_parameter("velocity_lookup", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        gain_scheduling_velocity_lookup_ = get_parameter("velocity_lookup").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'velocity_lookup\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'velocity_lookup\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'velocity_lookup\' is not set, using default values.");
    }

    // Feed Forward Acceleration Gain
    declare_parameter("feed_forward_acceleration_gain", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        vec_feed_forward_gain_acceleration_ = get_parameter("feed_forward_acceleration_gain").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'feed_forward_acceleration_gain\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'feed_forward_acceleration_gain\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'feed_forward_acceleration_gain\' is not set, using default values.");
    }

    // Feed Forward Steering Angle Gain
    declare_parameter("feed_forward_steering_angle", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        vec_feed_forward_gain_steering_angle_ = get_parameter("feed_forward_steering_angle").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'feed_forward_steering_angle\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'feed_forward_steering_angle\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'feed_forward_steering_angle\' is not set, using default values.");
    }

    // dv P Gain
    declare_parameter("dv_p", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dv_p_ = get_parameter("dv_p").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'dv_p\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dv_p\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dv_p\' is not set, using default values.");
    }

    // dv I Gain
    declare_parameter("dv_i", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dv_i_ = get_parameter("dv_i").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'dv_i\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dv_i\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dv_i\' is not set, using default values.");
    }

    // dv D Gain
    declare_parameter("dv_d", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dv_d_ = get_parameter("dv_d").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'dv_d\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dv_d\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dv_d\' is not set, using default values.");
    }

    // dy P Gain
    declare_parameter("dy_p", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dy_p_ = get_parameter("dy_p").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'dy_p\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dy_p\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dy_p\' is not set, using default values.");
    }

    // dy I Gain
    declare_parameter("dy_i", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dy_i_ = get_parameter("dy_i").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'dy_i\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dy_i\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dy_i\' is not set, using default values.");
    }

    // dy D Gain
    declare_parameter("dy_d", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dy_d_ = get_parameter("dy_d").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(),  "Parameter-Vector \'dy_d\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dy_d\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dy_d\' is not set, using default values.");
    }

    // dpsi P Gain
    declare_parameter("dpsi_p", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dpsi_p_ = get_parameter("dpsi_p").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'dpsi_p\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dpsi_p\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dpsi_p\' is not set, using default values.");
    }

    // dpsi I Gain
    declare_parameter("dpsi_i", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dpsi_i_ = get_parameter("dpsi_i").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'dpsi_i\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dpsi_i\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dpsi_i\' is not set, using default values.");
    }

    // dpsi D Gain
    declare_parameter("dpsi_d", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    try {
        dpsi_d_ = get_parameter("dpsi_d").as_double_array();
        RCLCPP_INFO_STREAM(get_logger(), "Parameter-Vector \'dpsi_d\' set succesfully.");
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dpsi_d\' is not set correctly, using default values.");
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(get_logger(), "Parameter-Vector \'dpsi_d\' is not set, using default values.");
    }
}

//update the actual vehicle state
void TrajectoryControl::VehicleStateCallback(const perception_interfaces::msg::EgoData::ConstPtr &msg)
{
    cur_vehicle_state_ = *msg;
}

//update the current trajectory
void TrajectoryControl::TrajectoryCallback(const trajectory_interfaces::msg::Trajectory::ConstPtr &msg)
{
    cur_trajectory_ = *msg;
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
    trajectory_interfaces::msg::Trajectory dummy_trj;
    cur_trajectory_=dummy_trj;
    perception_interfaces::msg::EgoData dummy_state;
    cur_vehicle_state_=dummy_state;
    ResetOdometry();
}

void TrajectoryControl::setControllerGains()
{
    double velocity = perception_interfaces::object_access::getVelLon(cur_vehicle_state_);
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
    if(cur_trajectory_.standstill)
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
    vehicle_ctrl_pub_->publish(vhcl_ctrl_output_);
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
    if (trajectory_interfaces::trajectory_access::getSamplePointSize(cur_trajectory_) == 0)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Input Trajctory is empty!");
        return false;
    }

    return true;
}

bool TrajectoryControl::TrjDataProc()
{
    // Derive State Vectors
    std::vector<double> TIME, V, A, Y, THETA, KAPPA;
    int n_samples = trajectory_interfaces::trajectory_access::getSamplePointSize(cur_trajectory_);
    for(int i=0; i<n_samples; i++){
        TIME.push_back(trajectory_interfaces::trajectory_access::getT(cur_trajectory_, i));
        V.push_back(trajectory_interfaces::trajectory_access::getV(cur_trajectory_, i));
        Y.push_back(trajectory_interfaces::trajectory_access::getY(cur_trajectory_, i));
        if(cur_trajectory_.type_id==trajectory_interfaces::DRIVABLE::TYPE_ID)
        {
            A.push_back(trajectory_interfaces::trajectory_access::getA(cur_trajectory_, i));
            THETA.push_back(trajectory_interfaces::trajectory_access::getTheta(cur_trajectory_, i));
            KAPPA.push_back(trajectory_interfaces::trajectory_access::getKappa(cur_trajectory_, i));
        }
        else
        {
            // To-Do Fill A, THETA and KAPPA with finite-differences
            RCLCPP_ERROR_STREAM(get_logger(), "trajectory_interfaces::DRIVABLE-Type is currently not supported!");
            return false;
        }
    }

    //calculate desired interpolation time for longitudinal values
    double des_time = (now() - cur_trajectory_.header.stamp).seconds() + lon_t_lookahead_;
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
    double yawRate = perception_interfaces::object_access::getYawRate(cur_vehicle_state_);
    double velocity = perception_interfaces::object_access::getVelLon(cur_vehicle_state_);
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

    double velocity = perception_interfaces::object_access::getVelLon(cur_vehicle_state_);
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

    if (perception_interfaces::object_access::getStandstill(cur_vehicle_state_)) //Standstill-Situation
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
    double velocity = perception_interfaces::object_access::getVelLon(cur_vehicle_state_);
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
