#include "../include/visualizer.hpp"

PX4Visualizer::PX4Visualizer() : Node("visualizer") {
    // Initialize subscribers and publishers
    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
    auto qos_profile = rclcpp::SystemDefaultsQoS();



    attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        qos_profile,
        std::bind(&PX4Visualizer::vehicleAttitudeCallback, this, std::placeholders::_1));
    local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        qos_profile,
        std::bind(&PX4Visualizer::vehicleLocalPositionCallback, this, std::placeholders::_1));
    setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint",
        qos_profile,
        std::bind(&PX4Visualizer::trajectorySetpointCallback, this, std::placeholders::_1));

    vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/visualizer/vehicle_pose",
        qos_profile);
    vehicle_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualizer/vehicle_velocity",
        qos_profile);
    vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/visualizer/vehicle_path",
        qos_profile);
    setpoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/visualizer/setpoint_path",
        qos_profile);

    // Initialize timer
    auto timer_callback = std::bind(&PX4Visualizer::cmdloopCallback, this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);
}

void PX4Visualizer::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    // Convert quaternion to attitude array
    vehicle_attitude_[0] = msg->q[0];
    vehicle_attitude_[1] = msg->q[1];
    vehicle_attitude_[2] = -msg->q[2]; // NED to ENU conversion
    vehicle_attitude_[3] = -msg->q[3]; // NED to ENU conversion
}

void PX4Visualizer::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    // Convert NED to ENU and update local position and velocity arrays
    vehicle_local_position_[0] = msg->x;
    vehicle_local_position_[1] = -msg->y; // NED to ENU conversion
    vehicle_local_position_[2] = -msg->z; // NED to ENU conversion
    vehicle_local_velocity_[0] = msg->vx;
    vehicle_local_velocity_[1] = -msg->vy; // NED to ENU conversion
    vehicle_local_velocity_[2] = -msg->vz; // NED to ENU conversion
}

void PX4Visualizer::trajectorySetpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
    // Update setpoint position array
    setpoint_position_[0] = msg->position[0];
    setpoint_position_[1] = -msg->position[1]; // NED to ENU conversion
    setpoint_position_[2] = -msg->position[2]; // NED to ENU conversion
}

void PX4Visualizer::cmdloopCallback() {
    // Publish vehicle pose
    vehicle_pose_pub_->publish(vectorToPoseMsg("map", vehicle_local_position_, vehicle_attitude_));

    // Publish vehicle path
    vehicle_path_msg_.header.stamp = this->get_clock()->now();
    vehicle_path_msg_.header.frame_id = "map";
    vehicle_path_msg_.poses.push_back(vectorToPoseMsg("map", vehicle_local_position_, vehicle_attitude_));
    vehicle_path_pub_->publish(vehicle_path_msg_);

    // Publish setpoint path
    setpoint_path_msg_.header.stamp = this->get_clock()->now();
    setpoint_path_msg_.header.frame_id = "map";
    setpoint_path_msg_.poses.push_back(vectorToPoseMsg("map", setpoint_position_, vehicle_attitude_));
    setpoint_path_pub_->publish(setpoint_path_msg_);

    // Publish velocity marker
    vehicle_vel_pub_->publish(createArrowMarker(1, vehicle_local_position_, vehicle_local_velocity_));
}

geometry_msgs::msg::PoseStamped PX4Visualizer::vectorToPoseMsg(const std::string& frame_id, const std::array<double, 3>& position, const std::array<double, 4>& attitude) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose.orientation.w = attitude[0];
    pose_msg.pose.orientation.x = attitude[1];
    pose_msg.pose.orientation.y = attitude[2];
    pose_msg.pose.orientation.z = attitude[3];
    pose_msg.pose.position.x = position[0];
    pose_msg.pose.position.y = position[1];
    pose_msg.pose.position.z = position[2];
    return pose_msg;
}

visualization_msgs::msg::Marker PX4Visualizer::createArrowMarker(int id, const std::array<double, 3>& tail, const std::array<double, 3>& vector) {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.0;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    double dt = 0.3;
    geometry_msgs::msg::Point tail_point, head_point;
    tail_point.x = tail[0];
    tail_point.y = tail[1];
    tail_point.z = tail[2];
    head_point.x = tail[0] + dt * vector[0];
    head_point.y = tail[1] + dt * vector[1];
    head_point.z = tail[2] + dt * vector[2];
    marker.points.push_back(tail_point);
    marker.points.push_back(head_point);
    return marker;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4Visualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
