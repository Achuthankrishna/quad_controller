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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <map>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sys/ioctl.h>
using namespace std::chrono_literals;

class TeleopTwistKeyboard : public rclcpp::Node
{
public:
    TeleopTwistKeyboard()
        : Node("teleop_twist_keyboard"), speed_(0.5), turn_(0.2), x_(0.0), y_(0.0), z_(0.0), th_(0.0), arm_toggle_(false)
    {
         RCLCPP_INFO(get_logger(), R"(
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
        )");
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/offboard_velocity_cmd", 10);
        arm_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arm_message", 10);

        settings_ = saveTerminalSettings();

        this->timer_ = this->create_wall_timer(
            100ms, std::bind(&TeleopTwistKeyboard::keyboardLoop, this));
    }

private:
    void keyboardLoop()
    {
        if (kbhit())
        {
            char key = getchar();
            std::cout << "Key pressed: " << key << std::endl;

            if (moveBindings_.find(key) != moveBindings_.end())
            {
                RCLCPP_INFO(this->get_logger(), "Pressed key: %c", key);
                RCLCPP_INFO(this->get_logger(), "GETTING PRESSED");
                x_ = std::get<0>(moveBindings_.at(key)); 
                y_ = std::get<1>(moveBindings_.at(key)); 
                z_ = std::get<2>(moveBindings_.at(key)); 
                th_ = std::get<3>(moveBindings_.at(key));
            }
            else
            {
                x_ = 0.0;
                y_ = 0.0;
                z_ = 0.0;
                th_ = 0.0;
             
            }
            if (key == ' ')
            {
                arm_toggle_ = !arm_toggle_;
                auto arm_msg = std_msgs::msg::Bool();
                arm_msg.data = arm_toggle_;
                arm_pub_->publish(arm_msg);
                RCLCPP_INFO(this->get_logger(), "Arm toggle is now: %s", arm_toggle_ ? "true" : "false");
            }

            auto twist = geometry_msgs::msg::Twist();
            x_val_ = (x_ * speed_) + x_val_;
            y_val_ = (y_ * speed_) + y_val_;
            z_val_ = (z_ * speed_) + z_val_;
            yaw_val_ = (th_ * turn_) + yaw_val_;

            std::cout << "Velocities - x: " << x_val_ << ", y: " << y_val_ << ", z: " << z_val_ << ", yaw: " << yaw_val_ << std::endl;

            twist.linear.x = x_val_;
            twist.linear.y = y_val_;
            twist.linear.z = z_val_;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = yaw_val_;


            std::cout << "Twist message - linear: [" << twist.linear.x << ", " << twist.linear.y << ", " << twist.linear.z << "], angular: [" << twist.angular.x << ", " << twist.angular.y << ", " << twist.angular.z << "]" << std::endl;

            pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "X: %f   Y: %f   Z: %f   Yaw: %f", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z);
        }
    }

    std::shared_ptr<termios> saveTerminalSettings()
    {
        auto old_settings = std::make_shared<termios>();
        tcgetattr(STDIN_FILENO, old_settings.get());
        return old_settings;
    }

    bool kbhit()
    {
        termios term;
        tcgetattr(STDIN_FILENO, &term);
        termios term2 = term;
        term2.c_lflag &= ~ICANON;
        tcsetattr(STDIN_FILENO, TCSANOW, &term2);

        int bytesWaiting;
        ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);
        tcsetattr(STDIN_FILENO, TCSANOW, &term);
        return bytesWaiting > 0;
    }

    std::string msg_;
    std::shared_ptr<termios> settings_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;
    float speed_;
    float turn_;
    float x_;
    float y_;
    float z_;
    float th_;
    float x_val_;
    float y_val_;
    float z_val_;
    float yaw_val_;
    bool arm_toggle_;
    const std::map<char, std::tuple<float, float, float, float>> moveBindings_ = {
        {'w', {0, 0, 1, 0}},       
        {'s', {0, 0, -1, 0}},      
        {'a', {0, 0, 0, -1}},      
        {'d', {0, 0, 0, 1}},       
        {65, {0, 1, 0, 0}},        
        {66, {0, -1, 0, 0}},       
        {67, {-1, 0, 0, 0}},       
        {68, {1, 0, 0, 0}},        
    };

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTwistKeyboard>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}