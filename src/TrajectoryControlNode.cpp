/**
 * @file TrajectoryControlNode.cpp
 * @author Guido Küppers
 * @brief  ROS-Node for trajectory control.
 */
#pragma once

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
    this->dv_pid = new PID(0.0, 0.0, 0.0);

    //Initialize dy-PID
    this->dy_pid = new PID(0.0, 0.0, 0.0);

    //Initialize dpsi-PID
    this->dpsi_pid = new PID(0.0, 0.0, 0.0);

    //Set Initial output values
    vhcl_ctrl_output_.drive.steering_angle = 0.0;
    vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
    vhcl_ctrl_output_.drive.speed = 0.0;
    vhcl_ctrl_output_.drive.acceleration = 0.0;
    vhcl_ctrl_output_.drive.jerk = 0.0;

    this->ResetOdometry();

    //Initialization of the cyclic vehicle-control timer; the callback VehicleCtrlCycle will be called every 0.01s e.g. with 100Hz.
    last_time_=now();
    vhcl_ctrl_timer_ = create_wall_timer(std::chrono::duration<double>(control_frequency_), std::bind(&TrajectoryControl::VehicleCtrlCycle, this));
}

TrajectoryControl::~TrajectoryControl()
{
}

void TrajectoryControl::loadParameters() {

    // General parameters
    // Control cycle frequency
    this->declare_parameter("control_frequency", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'control_frequency\' set to: " << control_frequency_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'control_frequency\' is not set correctly, using default value: " << control_frequency_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'control_frequency\' is not set, using default value: " << control_frequency_);
    }

    // Constant vehicle parameters
    // Wheelbase
    this->declare_parameter("wheelbase", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'wheelbase\' set to: " << wheelbase_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'wheelbase\' is not set correctly, using default value: " << wheelbase_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'wheelbase\' is not set, using default value: " << wheelbase_);
    }

    // Self-Steer-Gradient
    this->declare_parameter("selfsteergradient", rclcpp::ParameterType::PARAMETER_DOUBLE);
    try {
        self_st_gradient_ = this->get_parameter("selfsteergradient").as_double();
        RCLCPP_INFO_STREAM(this->get_logger(), "Parameter \'selfsteergradient\' set to: " << self_st_gradient_);
    } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'selfsteergradient\' is not set correctly, using default value: " << self_st_gradient_);
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Parameter \'selfsteergradient\' is not set, using default value: " << self_st_gradient_);
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
    this->ResetOdometry();
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
    dy_pid->Reset();
    dpsi_pid->Reset();
    dv_pid->Reset();
    vhcl_ctrl_output_.drive.steering_angle = 0.0;
    vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
    vhcl_ctrl_output_.drive.speed = 0.0;
    vhcl_ctrl_output_.drive.acceleration = 0.0;
    vhcl_ctrl_output_.drive.jerk = 0.0;
    trajectory_interfaces::msg::Trajectory dummy_trj;
    cur_trajectory_=dummy_trj;
    perception_interfaces::msg::EgoData dummy_state;
    cur_vehicle_state_=dummy_state;
    this->ResetOdometry();
}

//perform the vehicle control cycle
void TrajectoryControl::VehicleCtrlCycle()
{
    if(last_time_ > now())
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Resetting controller because of Jump-Back in time!");
        this->ResetController();
    }
    last_time_=now();
    if (!this->InputSanityCheck()) //some inputs are not ok
    {
        //don't do anything
        return;
    }

    // Standstill signal?
    if(cur_trajectory_.standstill)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Standstill.");
        vhcl_ctrl_output_.drive.steering_angle = 0.0;
        vhcl_ctrl_output_.drive.steering_angle_velocity = 0.0;
        vhcl_ctrl_output_.drive.speed = 0.0;
        vhcl_ctrl_output_.drive.acceleration = 0.0;
        vhcl_ctrl_output_.drive.jerk = 0.0;
        dy_pid->Reset();
        dpsi_pid->Reset();
        dv_pid->Reset();
    }
    else
    {
        if(!TrjDataProc())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Processing of input Trajectory failed!");
            return;
        }
        vhcl_ctrl_output_.drive.steering_angle = this->LateralControl();
        vhcl_ctrl_output_.drive.acceleration = this->LongitudinalControl();
        if(std::isnan(vhcl_ctrl_output_.drive.steering_angle))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Steering Angle Output Value isNaN!");
            vhcl_ctrl_output_.drive.steering_angle=0.0;
            dy_pid->Reset();
            dpsi_pid->Reset();
            return;
        }
        if(std::isnan(vhcl_ctrl_output_.drive.acceleration))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Target Acceleration Output Value isNaN!");
            vhcl_ctrl_output_.drive.acceleration=0.0;
            dv_pid->Reset();
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
        RCLCPP_ERROR_STREAM(this->get_logger(), "EgoState-Data outdated!");
        return false;
    }
    if (trajectory_interfaces::trajectory_access::getSamplePointSize(cur_trajectory_) == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Input Trajctory is empty!");
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
            RCLCPP_ERROR_STREAM(this->get_logger(), "trajectory_interfaces::DRIVABLE-Type is currently not supported!");
            return false;
        }
    }

    //calculate desired interpolation time for longitudinal values
    double des_time = (now() - cur_trajectory_.header.stamp).seconds() + lon_t_lookahead_;
    //interpolate longitudinal target values
    v_tgt_ = this->InterpolateTgtValue(TIME, V, des_time, n_samples);
    a_tgt_ = this->InterpolateTgtValue(TIME, A, des_time, n_samples);

    //switch to desired interpolation time for lateral values
    des_time = des_time - lon_t_lookahead_ + lat_t_lookahead_;
    //interpolate lateral target values
    y_tgt_ = this->InterpolateTgtValue(TIME, Y, des_time, n_samples);
    psi_tgt_ = this->InterpolateTgtValue(TIME, THETA, des_time, n_samples);
    kappa_tgt_ = this->InterpolateTgtValue(TIME, KAPPA, des_time, n_samples);

    //CalcOdometry
    double dt = (now() - vhcl_ctrl_output_.header.stamp).seconds();
    this->CalcOdometry(dt); //Cyclic Control

    dy_ = odom_dy_ - y_tgt_;
    dpsi_ = odom_dpsi_ - psi_tgt_;
    return true;
}

double TrajectoryControl::InterpolateTgtValue(std::vector<double> time_array, std::vector<double> val_array, double desired_time, unsigned int num_elements)
{
    if (desired_time < 0.0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Desired Time for Trajectory Interpolation is negative!");
        return 0.0;
    }
    if(num_elements <=1)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Num Elements for Trajectory Interpolation is 1!");
        return 0.0;
    }

    //go through array and search for sampling points
    int i;
    for(i = 0; i < num_elements; i++)
    {
        if (time_array[i] < desired_time)
        {
            continue;
        }
        else if (time_array[i] == desired_time)
        {
            return val_array[i];
        }
        else
        {
            break;
        }
    }
    if (i < num_elements - 1)
    {
        return val_array[i - 1] + ((val_array[i] - val_array[i - 1]) / (time_array[i] - time_array[i - 1])) * (desired_time - time_array[i - 1]);
    }
    else
    {
        return val_array[i] + ((val_array[i] - val_array[i - 1]) / (time_array[i] - time_array[i - 1])) * (desired_time - time_array[i]);
    }
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
    double w_psi = dy_pid->Calc(e_y, dt);
    double e_psi = w_psi - dpsi_;
    double psi_dot_des = dpsi_pid->Calc(e_psi, dt);

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
        dy_pid->Reset();
        dpsi_pid->Reset();
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
        dy_pid->Reset();
        dpsi_pid->Reset();
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Steering-Angle limited!");
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
        dy_pid->Reset();
        dpsi_pid->Reset();
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Steering-rate limited!");
    }

    return st_ang;
}

double TrajectoryControl::LongitudinalControl()
{
    double dt = (now() - vhcl_ctrl_output_.header.stamp).seconds();
    double velocity = perception_interfaces::object_access::getVelLon(cur_vehicle_state_);
    double w_v = v_tgt_;
    double e_v = w_v - velocity;
    double a_fb_v = dv_pid->Calc(e_v, dt);

    double a_ctrl = a_fb_v + a_tgt_ * feed_forward_gain_acceleration_;

    //Limit desired acceleration
    if (a_ctrl > lon_max_acc_)
    {
        a_ctrl = lon_max_acc_;
        dv_pid->Reset();
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Longitudinal acceleration limited!");
    }
    else if (a_ctrl < lon_min_acc_)
    {
        a_ctrl = lon_min_acc_;
        dv_pid->Reset();
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Longitudinal acceleration limited!");
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
        //dv_pid->Reset();
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Longitudinal jerk limited!");
    }
    vhcl_ctrl_output_.drive.speed = v_tgt_;
    return a_ctrl;
}
