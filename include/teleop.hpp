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
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

class TeleopTwistKeyboard : public rclcpp::Node {
public:
    TeleopTwistKeyboard();    

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;

    char getKey();
    void publishTwist(const std::string& key);
    bool arm_toggle_;
    void armToggleCallback(const std_msgs::msg::Bool::SharedPtr msg);
    
};