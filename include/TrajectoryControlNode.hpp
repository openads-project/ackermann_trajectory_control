/**
 * @file TrajectoryControlNode.hpp
 * @author Guido Küppers
 * @brief  ROS-Node for trajectory control.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <PID.hpp>

// Input Messages
#include <trajectory_interfaces/msg/trajectory.hpp>
#include <trajectory_interfaces/msg/drivable.hpp>
#include <trajectory_interfaces/msg/reference.hpp>

#include <perception_interfaces/msg/ego_data.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

// Output Messages
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class TrajectoryControl : public rclcpp::Node
{
public:
    TrajectoryControl();
    ~TrajectoryControl();

private:
    void loadParameters();
    void VehicleStateCallback(const perception_interfaces::msg::EgoData::ConstPtr &msg);
    void TrajectoryCallback(const trajectory_interfaces::msg::Trajectory::ConstPtr &msg);
    void ClockCallback(const rosgraph_msgs::msg::Clock::ConstPtr &msg);
    void VehicleCtrlCycle();
    bool InputSanityCheck();
    bool TrjDataProc();
    double InterpolateTgtValue(std::vector<double> time_array, std::vector<double> val_array, double desired_time, unsigned int num_elements);
    void CalcOdometry(double dt);
    void ResetOdometry();
    double LateralControl();
    double LongitudinalControl();
    void ResetController();

    rclcpp::Subscription<perception_interfaces::msg::EgoData>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<trajectory_interfaces::msg::Trajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr vehicle_ctrl_pub_;

    rclcpp::TimerBase::SharedPtr vhcl_ctrl_timer_;

    perception_interfaces::msg::EgoData cur_vehicle_state_;
    trajectory_interfaces::msg::Trajectory cur_trajectory_;

    //TrajectoryControl Parameters
    double control_frequency_ = 0.01;

    double lat_t_lookahead_ = 0.1;
    double lon_t_lookahead_ = 0.1;

    double lon_max_acc_ = 3.5;
    double lon_min_acc_ = -5.0; //be sure that this value is negative
    double lon_max_jerk_ = 5.0;

    double lat_max_st_ang_ = 28.0*M_PI/180.0;
    double lat_max_st_rate_ = 56.0*M_PI/180.0;

    double feed_forward_gain_steering_angle_ = 1.0;
    double feed_forward_gain_acceleration_ = 1.0;

    //Vehicle Parameters
    double wheelbase_ = 2.711;
    double self_st_gradient_ = 0.002917853041365;

    //TrajectoryControl Variables
    double a_tgt_;
    double a_tgt_dv_;
    double v_tgt_;
    double y_tgt_;
    double psi_tgt_;
    double kappa_tgt_;

    //Control Deviations
    double dpsi_;
    double dy_;
    double dv_;

    //Controller
    PID *dv_pid;
    PID *dy_pid;
    PID *dpsi_pid;

    //Control Output
    ackermann_msgs::msg::AckermannDriveStamped vhcl_ctrl_output_;

    rosgraph_msgs::msg::Clock last_clock_;

    //Odometry
    double odom_dy_ = 0.0;
    double odom_dpsi_ = 0.0;

}; //class TrajectoryControl
