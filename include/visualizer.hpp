#pragma once

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "visualization_msgs/msg/marker.hpp"

class PX4Visualizer : public rclcpp::Node {
public:
    PX4Visualizer();

private:
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void trajectorySetpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);
    void cmdloopCallback();

    geometry_msgs::msg::PoseStamped vectorToPoseMsg(const std::string& frame_id, const std::array<double, 3>& position, const std::array<double, 4>& attitude);
    visualization_msgs::msg::Marker createArrowMarker(int id, const std::array<double, 3>& tail, const std::array<double, 3>& vector);

    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;

    std::array<double, 4> vehicle_attitude_;
    std::array<double, 3> vehicle_local_position_;
    std::array<double, 3> vehicle_local_velocity_;
    std::array<double, 3> setpoint_position_;
    nav_msgs::msg::Path vehicle_path_msg_;
    nav_msgs::msg::Path setpoint_path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};
