#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class TorqueServoController : public rclcpp::Node
{
public:
    TorqueServoController() : Node("torque_Pgain_simple_ctl")
    {
        // --- Parameters ---
        // Scale factor to convert angular error into physical torque (Nm)
        error_to_torque_gain_ = 5.0;  
        // Hard safety limits to prevent unstable physics simulation bursts
        max_torque_limit_ = 10.0;     

        // --- Subscriptions ---
        // Subscribes to the sensor publishing your angular offset error
        sensor_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sensor/angular_error", 
            10, 
            std::bind(&TorqueServoController::sensor_callback, this, std::placeholders::_1)
        );

        // --- Publishers ---
        // Publishes the target torque to the ros2_control effort controller
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/servo_torque_controller/commands", 
            10
        );

        RCLCPP_INFO(this->get_logger(), "C++ Torque Servo Feedback Controller Node Initialized!");
    }

private:
    void sensor_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // 1. Read the angular offset error from your sensor input
        double angular_error = msg->data;

        // 2. Map the digital command/offset to an equivalent torque load
        double target_torque = angular_error * error_to_torque_gain_;

        // 3. Apply safety constraints (clamping limits)
        target_torque = std::clamp(target_torque, -max_torque_limit_, max_torque_limit_);

        // 4. Pack the command into a Float64MultiArray for ros2_control
        auto command_msg = std_msgs::msg::Float64MultiArray();
        command_msg.data.push_back(target_torque);

        // 5. Publish to Gazebo
        torque_pub_->publish(command_msg);

        // Optional debug tracking to terminal window
        RCLCPP_DEBUG(this->get_logger(), "Error: %.3f rad | Commanded Torque: %.3f Nm", 
                     angular_error, target_torque);
    }

    // Member variables for the publisher, subscriber, and variables
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sensor_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    
    double error_to_torque_gain_;
    double max_torque_limit_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueServoController>());
    rclcpp::shutdown();
    return 0;
}
