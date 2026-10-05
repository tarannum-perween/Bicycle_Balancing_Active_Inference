// Note: to run the node from the command line, use the following command:
// ros2 run pid_cpp torque_dynamic_pid --ros-args -p kp:=1.0 -p ki:=0.0 -p kd:=0.0 -p nominal_torque:=0.0 -p max_torque_limit:=5.0
// or, if you have a YAML file with the parameters, use:
// ros2 run pid_cpp torque_dynamic_pid --ros-args --params-file <path_to_yaml_file>
//
// Here is an example of a YAML file with the parameters:
/* 
torque_servo_controller:
  ros__parameters:
    use_sim_time: true
    
    # PID Gains for mapping the offset error to torque
    kp: 5.0
    ki: 0.1
    kd: 0.05
    
    # Nominal torque baseline (e.g., constant gravity compensation)
    nominal_torque: 0.2
    
# Motor Physical Characteristics
    stall_torque: 12.0       # Max torque at 0 rad/s speed (Nm)
    no_load_speed: 50.0      # Max speed at 0 Nm torque (rad/s)
    absolute_max_torque: 10.0 # Upper safety ceiling regardless of voltage (Nm)

*/

#include <memory>
#include <algorithm>
#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class TorqueServoController : public rclcpp::Node
{
public:
    TorqueServoController() : Node("torque_servo_controller")
    {
        // 1. Declare and initialize parameters with default fallback values
        this->declare_parameter<double>("kp", 1.0);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.0);
        this->declare_parameter<double>("nominal_torque", 0.0);
        this->declare_parameter<double>("max_torque_limit", 5.0);

        // 2. Fetch the initial values from the YAML file
        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);
        this->get_parameter("nominal_torque", nominal_torque_);
        this->get_parameter("max_torque_limit", max_torque_limit_);

        // 3. Set up a callback handler to watch for live parameter changes
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TorqueServoController::on_parameter_change, this, std::placeholders::_1)
        );

        // Clear PID history variables
        latest_angular_error_ = 0.0;
        integral_error_ = 0.0;
        previous_error_ = 0.0;

        // --- Subscriptions ---
        sensor_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sensor/angular_error", 
            10, 
            std::bind(&TorqueServoController::sensor_callback, this, std::placeholders::_1)
        );

        // --- Publishers ---
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/servo_torque_controller/commands", 
            10
        );

        // --- 1000 Hz Fixed Frequency Control Timer ---
        control_timer_ = this->create_wall_timer(
            1ms, 
            std::bind(&TorqueServoController::control_loop_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Dynamic PID Torque Controller Node Initialized!");
    }

private:
    void sensor_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        latest_angular_error_.store(msg->data);
    }

    void control_loop_callback()
    {
        // Define the exact time step (1 millisecond = 0.001 seconds)
        const double dt = 0.001; 

        // Fetch the newest error value safely
        double current_error = latest_angular_error_.load();

        // --- PID Core Calculations ---
        // Proportional term
        double p_term = kp_ * current_error;

        // Integral term (accumulated over time)
        integral_error_ += current_error * dt;
        double i_term = ki_ * integral_error_;

        // Derivative term (rate of error change)
        double error_derivative = (current_error - previous_error_) / dt;
        double d_term = kd_ * error_derivative;

        // Save state for the next 1ms iteration loop
        previous_error_ = current_error;

        // Combine terms and add the baseline nominal torque loading offset
        double target_torque = p_term + i_term + d_term + nominal_torque_;

        // Force anti-windup safety clamp on the integral error if total torque maxes out
        if (std::abs(target_torque) > max_torque_limit_) {
            // Roll back the integration step to prevent massive error build-up
            integral_error_ -= current_error * dt; 
        }

        // Clamp final output torque cleanly within boundaries
        target_torque = std::clamp(target_torque, -max_torque_limit_, max_torque_limit_);

        // Pack and publish to Gazebo Classic effort interface
        auto command_msg = std_msgs::msg::Float64MultiArray();
        command_msg.data.push_back(target_torque);
        torque_pub_->publish(command_msg);
    }

    // Handles live parameters tweaks during runtime via terminal or rqt
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "kp") {
                kp_ = param.as_double();
            } else if (param.get_name() == "ki") {
                ki_ = param.as_double();
                integral_error_ = 0.0; // Reset integration accumulator when tuning ki
            } else if (param.get_name() == "kd") {
                kd_ = param.as_double();
            } else if (param.get_name() == "nominal_torque") {
                nominal_torque_ = param.as_double();
            } else if (param.get_name() == "max_torque_limit") {
                max_torque_limit_ = param.as_double();
            }
        }
        return result;
    }

    // Communication Interfaces
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sensor_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Thread-safe shared variable
    std::atomic<double> latest_angular_error_;

    // PID History States
    double integral_error_;
    double previous_error_;

    // Tunable Variables
    double kp_;
    double ki_;
    double kd_;
    double nominal_torque_;
    double max_torque_limit_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueServoController>());
    rclcpp::shutdown();
    return 0;
}
