/**
 * @file teleop.hpp
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "../include/teleop.hpp"

#include <iostream>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
//For Jyostick
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

const std::map<char, std::array<double, 4>> moveBindings = {
    {'w', {1.0, 0.0, 0.0, 0.0}},   // Up
    {'s', {-1.0, 0.0, 0.0, 0.0}},  // Down
    {'a', {0.0, 0.0, 0.0, 1.0}},   // Yaw Left
    {'d', {0.0, 0.0, 0.0, -1.0}},  // Yaw Right
    {'\x1b', {0.0, 0.0, 0.0, 0.0}} // Stop
};

TeleopTwistKeyboard::TeleopTwistKeyboard()
    : Node("teleop_twist_keyboard"), arm_toggle_(false)
{
    // pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/offboard_velocity_cmd", 10);
    
    // arm_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arm_message", 10);
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/offboard_velocity_cmd", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    arm_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arm_message", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Subscribe to the topics with the appropriate QoS settings
    auto arm_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/arm_toggle", qos_profile,
        std::bind(&TeleopTwistKeyboard::armToggleCallback, this, std::placeholders::_1)
    );
    // // Subscribe to the arm toggle topic
    // auto arm_sub = this->create_subscription<std_msgs::msg::Bool>(
    //     "/arm_toggle", 10, std::bind(&TeleopTwistKeyboard::armToggleCallback, this, std::placeholders::_1)
    // );

    std::cout << R"(
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
)" << std::endl;
}

TeleopTwistKeyboard::~TeleopTwistKeyboard()
{
    // Your cleanup code
}

void TeleopTwistKeyboard::publishTwist(const std::string& key)
{
    geometry_msgs::msg::Twist twist;

    if (moveBindings.find(key[0]) != moveBindings.end()) {
        auto move = moveBindings.at(key[0]);
        twist.linear.x = move[0];
        twist.linear.y = move[1];
        twist.linear.z = move[2];
        twist.angular.z = move[3];
    } else {
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.z = 0.0;
    }

    pub_->publish(twist);
}

void TeleopTwistKeyboard::armToggleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    arm_toggle_ = !arm_toggle_;

    auto arm_msg = std::make_shared<std_msgs::msg::Bool>();
    arm_msg->data = arm_toggle_;

    arm_pub_->publish(*arm_msg); // Dereference the shared pointer

    RCLCPP_INFO(get_logger(), "Arm toggle is now: %s", arm_toggle_ ? "true" : "false");
}


char TeleopTwistKeyboard::getKey()
{
    char key = 0;
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    key = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    if (key == 27) {
        getchar();
        key = getchar();
    }
    return key;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTwistKeyboard>();

    while (rclcpp::ok()) {
        char key = node->getKey();
        if (key == 3) {  // ASCII value for Ctrl+C
            break;
        }
        std::string key_str(1, key);
        node->publishTwist(key_str);
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
