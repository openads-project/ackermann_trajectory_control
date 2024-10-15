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
#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs/msg/drivable.hpp>
#include <trajectory_planning_msgs/msg/reference.hpp>

#include <perception_msgs/msg/ego_data.hpp>

// Output Messages
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>


template <typename C> struct is_vector : std::false_type {};    
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};    
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;

class TrajectoryControl : public rclcpp::Node
{
public:
    TrajectoryControl();
    ~TrajectoryControl();

protected:
    // ROS message parameters
    static const std::string kInputTopicEgoData;
    static const std::string kInputTopicTrajectory;
    static const std::string kOutputTopic;

private:

    // setup and parameter handling
    void setup();
    void loadParameters();
    template <typename T>
    void declareAndLoadParameter(const std::string &name, T &member_param, const std::string &description,
                                const bool add_to_auto_reconfigurable_params = true, const bool is_required = false,
                                const bool read_only = false, const std::optional<T> &from_value = std::nullopt,
                                const std::optional<T> &to_value = std::nullopt,
                                const std::optional<T> &step_value = std::nullopt,
                                const std::string &additional_constraints = "");
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    // callbacks
    void VehicleStateCallback(const perception_msgs::msg::EgoData::ConstPtr &msg);
    void TrajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::ConstPtr &msg);
    void VehicleCtrlCycle();

    void setControllerGains();
    bool InputSanityCheck();
    bool TrjDataProc();
    bool LinearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y);
    void CalcOdometry(double dt);
    void ResetOdometry();
    double LateralControl();
    double LongitudinalControl();
    void ResetController();

    rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr vehicle_ctrl_pub_;

    rclcpp::TimerBase::SharedPtr vhcl_ctrl_timer_;

    OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

    perception_msgs::msg::EgoData cur_vehicle_state_;
    trajectory_planning_msgs::msg::Trajectory cur_trajectory_;

    rclcpp::Time last_time_;

    // parameters
    std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>>
    auto_reconfigurable_params_;

    // TrajectoryControl Parameters
    std::string vehicle_frame_id_ = "base_link";
    std::string fixed_over_time_frame_id_ = "map";
    double control_frequency_ = 100.0;

    double lat_t_lookahead_ = 0.1;
    double lon_t_lookahead_ = 0.1;

    double lon_max_acc_ = 3.5;
    double lon_min_acc_ = -5.0; // make sure that this value is negative
    double lon_max_jerk_ = 5.0;

    double lat_max_st_ang_ = 28.0*M_PI/180.0;
    double lat_max_st_rate_ = 56.0*M_PI/180.0;

    // Vehicle Parameters
    double wheelbase_ = 2.711;
    double self_st_gradient_ = 0.002917853041365;

    // TrajectoryControl Variables
    double a_tgt_;
    double a_tgt_dv_;
    double v_tgt_;
    double y_tgt_;
    double psi_tgt_;
    double kappa_tgt_;

    // Control Deviations
    double dpsi_;
    double dy_;
    double dv_;

    // Controller
    PID *dv_pid_;
    PID *dy_pid_;
    PID *dpsi_pid_;

    std::vector<double> dv_p_ = {0.0, 0.0};
    std::vector<double> dv_i_ = {0.0, 0.0};
    std::vector<double> dv_d_ = {0.0, 0.0};
    std::vector<double> dy_p_ = {0.0, 0.0};
    std::vector<double> dy_i_ = {0.0, 0.0};
    std::vector<double> dy_d_ = {0.0, 0.0};
    std::vector<double> dpsi_p_ = {0.0, 0.0};
    std::vector<double> dpsi_i_ = {0.0, 0.0};
    std::vector<double> dpsi_d_ = {0.0, 0.0};
    std::vector<double> gain_scheduling_velocity_lookup_ = {-30.0, 30.0}; // velocity in m/s

    std::vector<double> vec_feed_forward_gain_steering_angle_ = {0.0, 0.0};
    std::vector<double> vec_feed_forward_gain_acceleration_ = {0.0, 0.0};

    double feed_forward_gain_steering_angle_;
    double feed_forward_gain_acceleration_;    

    // Control Output
    ackermann_msgs::msg::AckermannDriveStamped vhcl_ctrl_output_;

    // Odometry
    double odom_dy_ = 0.0;
    double odom_dpsi_ = 0.0;

}; // Class TrajectoryControl
