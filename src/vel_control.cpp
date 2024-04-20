#include "../include/vel_contro.hpp"

OffboardControl::OffboardControl() : Node("minimal_publisher") {
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(
            rmw_qos_profile_default.history,
            rmw_qos_profile_default.depth));
    qos_profile.reliability(rmw_qos_profile_default.reliability);
    qos_profile.durability(rmw_qos_profile_default.durability);

    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos_profile,
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
    offboard_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/offboard_velocity_cmd", qos_profile,
            std::bind(&OffboardControl::offboard_velocity_callback, this, std::placeholders::_1));
    attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos_profile,
            std::bind(&OffboardControl::attitude_callback, this, std::placeholders::_1));
    my_bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arm_message", qos_profile,
            std::bind(&OffboardControl::arm_message_callback, this, std::placeholders::_1));
    publisher_offboard_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos_profile);
    publisher_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/fmu/in/setpoint_velocity/cmd_vel_unstamped", qos_profile);
    publisher_trajectory_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos_profile);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos_profile);

    arm_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&OffboardControl::arm_timer_callback, this));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
            std::bind(&OffboardControl::cmdloop_callback, this));

    current_state=State::IDLE;
    last_state = current_state;
    myCnt = 0;
    arm_message = false;
    flightCheck = false;
    offboardMode = false;
    failsafe = false;
    yaw = 0.0;
    trueYaw = 0.0;
    nav_state = px4_msgs::msg::VehicleStatus_<std::allocator<void>>::NAVIGATION_STATE_MAX;
    arm_state = px4_msgs::msg::VehicleStatus_<std::allocator<void>>::ARMING_STATE_ARMED;
}

void OffboardControl::arm_message_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    arm_message = msg->data;
    RCLCPP_INFO(this->get_logger(), "Arm Message: %d", arm_message);
}
std::string stateToString(State state) {
    switch(state) {
        case State::IDLE:
            return "IDLE";
        case State::ARMING:
            return "ARMING";
        case State::TAKEOFF:
            return "TAKEOFF";
        case State::LOITER:
            return "LOITER";
        case State::OFFBOARD:
            return "OFFBOARD";
        default:
            return "UNKNOWN";
    }
}
void OffboardControl::arm_timer_callback() {
    switch(current_state) {
        case State::IDLE:
            if(flightCheck && arm_message == true) {
                current_state = State::ARMING;
                RCLCPP_INFO(this->get_logger(), "Arming");
            }
            break;
        case State::ARMING:
            if(!flightCheck) {
                current_state = State::IDLE;
                RCLCPP_INFO(this->get_logger(), "Arming, Flight Check Failed");
            } else if(arm_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED && myCnt > 10) {
                current_state = State::TAKEOFF;
                RCLCPP_INFO(this->get_logger(), "Arming, Takeoff");
            }
            arm();
            break;
        case State::TAKEOFF:
            if(!flightCheck) {
                current_state = State::IDLE;
                RCLCPP_INFO(this->get_logger(), "Takeoff, Flight Check Failed");
            } else if(nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF) {
                current_state = State::LOITER;
                RCLCPP_INFO(this->get_logger(), "Takeoff, Loiter");
            }
            arm();
            take_off();
            break;
        case State::LOITER:
            if(!flightCheck) {
                current_state = State::IDLE;
                RCLCPP_INFO(this->get_logger(), "Loiter, Flight Check Failed");
            } else if(nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
                current_state = State::OFFBOARD;
                RCLCPP_INFO(this->get_logger(), "Loiter, Offboard");
            }
            arm();
            break;
        case State::OFFBOARD:
            if(!flightCheck || arm_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED || failsafe == true) {
                current_state = State::IDLE;
                RCLCPP_INFO(this->get_logger(), "Offboard, Flight Check Failed");
            }
            state_offboard();
            break;
    }

    if(arm_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
        arm_message = false;
    }

    if(last_state != current_state) {
        last_state = current_state;
        std::string state_str = stateToString(current_state);
        RCLCPP_INFO(this->get_logger(), state_str.c_str());
        // RCLCPP_INFO(this->get_logger(),current_state);
    }

    myCnt++;
}

void OffboardControl::state_init() {
    myCnt = 0;
}

void OffboardControl::state_arming() {
    myCnt = 0;
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::state_takeoff() {
    myCnt = 0;
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 1.0, 0.0, 5.0); // param7 is altitude in meters
    RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}

void OffboardControl::state_loiter() {
    myCnt = 0;
    RCLCPP_INFO(this->get_logger(), "Loiter Status");
}

void OffboardControl::state_offboard() {
    myCnt = 0;
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    offboardMode = true;
}

void OffboardControl::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::take_off() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 1.0, 0.0, 5.0); // param7 is altitude in meters
    RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param7) {
    auto msg = std::make_unique<px4_msgs::msg::VehicleCommand>();
    msg->param1 = param1;
    msg->param2 = param2;
    msg->param7 = param7; // altitude value in takeoff command
    msg->command = command; // command ID
    msg->target_system = 1; // system which should execute the command
    msg->target_component = 1; // component which should execute the command, 0 for all components
    msg->source_system = 1; // system sending the command
    msg->source_component = 1; // component sending the command
    msg->from_external = true;
    msg->timestamp = rclcpp::Clock().now().nanoseconds() / 1000; // time in microseconds
    vehicle_command_publisher_->publish(std::move(msg));
}

void OffboardControl::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    if(msg->nav_state != nav_state) {
        RCLCPP_INFO(this->get_logger(), "NAV_STATUS: %d", msg->nav_state);
    }
    if(msg->arming_state != arm_state) {
        RCLCPP_INFO(this->get_logger(), "ARM STATUS: %d", msg->arming_state);
    }
    if(msg->failsafe != failsafe) {
        RCLCPP_INFO(this->get_logger(), "FAILSAFE: %d", msg->failsafe);
    }
    if(msg->pre_flight_checks_pass != flightCheck) {
        RCLCPP_INFO(this->get_logger(), "FlightCheck: %d", msg->pre_flight_checks_pass);
    }
    nav_state = msg->nav_state;
    arm_state = msg->arming_state;
    failsafe = msg->failsafe;
    flightCheck = msg->pre_flight_checks_pass;
}

void OffboardControl::offboard_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocity.x = -msg->linear.y;
    velocity.y = msg->linear.x;
    velocity.z = -msg->linear.z;
    yaw = msg->angular.z;
}

void OffboardControl::attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    float orientation_q[4] = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
    trueYaw = -(atan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                    1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])));
}

void OffboardControl::cmdloop_callback() {
    if(offboardMode == true) {
        px4_msgs::msg::OffboardControlMode offboard_msg;
        offboard_msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
        offboard_msg.position = false;
        offboard_msg.velocity = true;
        offboard_msg.acceleration = false;
        publisher_offboard_mode_->publish(offboard_msg);

        float cos_yaw = cos(trueYaw);
        float sin_yaw = sin(trueYaw);
        float velocity_world_x = (velocity.x * cos_yaw - velocity.y * sin_yaw);
        float velocity_world_y = (velocity.x * sin_yaw + velocity.y * cos_yaw);

        px4_msgs::msg::TrajectorySetpoint trajectory_msg;
        trajectory_msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
        trajectory_msg.velocity[0] = velocity_world_x;
        trajectory_msg.velocity[1] = velocity_world_y;
        trajectory_msg.velocity[2] = velocity.z;
        trajectory_msg.position[0] = NAN;
        trajectory_msg.position[1] = NAN;
        trajectory_msg.position[2] = NAN;
        trajectory_msg.acceleration[0] = NAN;
        trajectory_msg.acceleration[1] = NAN;
        trajectory_msg.acceleration[2] = NAN;
        trajectory_msg.yaw = NAN;
        trajectory_msg.yawspeed = yaw;

        publisher_trajectory_->publish(trajectory_msg);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto offboard_control = std::make_shared<OffboardControl>();
    rclcpp::spin(offboard_control);
    rclcpp::shutdown();
    return 0;
}
