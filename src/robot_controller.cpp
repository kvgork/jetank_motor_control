#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <chrono>
#include "motor.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        // Declare parameters with default values
        this->declare_parameter<int>("left_motor_pwm_pin", 18);
        this->declare_parameter<int>("left_motor_in1_pin", 23);
        this->declare_parameter<int>("left_motor_in2_pin", 24);
        this->declare_parameter<double>("left_motor_alpha", 1.0);
        
        this->declare_parameter<int>("right_motor_pwm_pin", 19);
        this->declare_parameter<int>("right_motor_in1_pin", 25);
        this->declare_parameter<int>("right_motor_in2_pin", 8);
        this->declare_parameter<double>("right_motor_alpha", 1.0);
        
        this->declare_parameter<double>("track_width", 0.11);
        this->declare_parameter<double>("max_linear_velocity", 1.0);
        this->declare_parameter<double>("max_angular_velocity", 2.0);
        
        // Get parameters
        int left_pwm_pin = this->get_parameter("left_motor_pwm_pin").as_int();
        int left_in1_pin = this->get_parameter("left_motor_in1_pin").as_int();
        int left_in2_pin = this->get_parameter("left_motor_in2_pin").as_int();
        double left_alpha = this->get_parameter("left_motor_alpha").as_double();
        
        int right_pwm_pin = this->get_parameter("right_motor_pwm_pin").as_int();
        int right_in1_pin = this->get_parameter("right_motor_in1_pin").as_int();
        int right_in2_pin = this->get_parameter("right_motor_in2_pin").as_int();
        double right_alpha = this->get_parameter("right_motor_alpha").as_double();
        
        track_width_ = this->get_parameter("track_width").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        
        // Initialize motors
        left_motor_ = std::make_unique<Motor>(left_pwm_pin, left_in1_pin, left_in2_pin, left_alpha);
        right_motor_ = std::make_unique<Motor>(right_pwm_pin, right_in1_pin, right_in2_pin, right_alpha);
        
        // Stop motors for safety
        stop();
        
        // Create subscriber for cmd_vel
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            10, 
            std::bind(&RobotController::cmdVelCallback, this, std::placeholders::_1)
        );
        
        // Create publisher for status
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_status", 10);
        
        // Create timer for periodic status updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotController::publishStatus, this)
        );
        
        // Create safety timer to stop motors if no cmd_vel received
        safety_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RobotController::safetyCheck, this)
        );
        
        last_cmd_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "Robot Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Track width: %.3f m", track_width_);
        RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.3f m/s", max_linear_velocity_);
        RCLCPP_INFO(this->get_logger(), "Max angular velocity: %.3f rad/s", max_angular_velocity_);
    }
    
    ~RobotController() {
        stop();
        RCLCPP_INFO(this->get_logger(), "Robot Controller shutting down");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Update last command time
        last_cmd_time_ = this->get_clock()->now();
        
        // Extract linear and angular velocities
        double lin_vel = msg->linear.x;
        double ang_vel = msg->angular.z;
        
        // Clamp velocities to maximum values
        lin_vel = std::max(-max_linear_velocity_, std::min(max_linear_velocity_, lin_vel));
        ang_vel = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, ang_vel));
        
        // Calculate differential drive motor commands
        // Left motor: linear velocity - (angular velocity * track_width / 2)
        // Right motor: linear velocity + (angular velocity * track_width / 2)
        double left_vel = lin_vel - (ang_vel * track_width_ / 2.0);
        double right_vel = lin_vel + (ang_vel * track_width_ / 2.0);
        
        // Normalize velocities to [-1, 1] range
        left_vel = left_vel / max_linear_velocity_;
        right_vel = right_vel / max_linear_velocity_;
        
        // Clamp to valid range
        left_vel = std::max(-1.0, std::min(1.0, left_vel));
        right_vel = std::max(-1.0, std::min(1.0, right_vel));
        
        // Set motor values
        left_motor_->setValue(left_vel);
        right_motor_->setValue(right_vel);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "CMD: lin=%.3f, ang=%.3f -> left=%.3f, right=%.3f", 
                    lin_vel, ang_vel, left_vel, right_vel);
    }
    
    void publishStatus() {
        auto message = std_msgs::msg::String();
        std::ostringstream ss;
        ss << "Robot Status - Left: " << left_motor_->getValue() 
           << ", Right: " << right_motor_->getValue();
        message.data = ss.str();
        status_publisher_->publish(message);
    }
    
    void safetyCheck() {
        auto now = this->get_clock()->now();
        auto time_since_last_cmd = now - last_cmd_time_;
        
        // If no command received for more than 1 second, stop motors
        if (time_since_last_cmd.seconds() > 1.0) {
            stop();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "No cmd_vel received for %.1f seconds, motors stopped", 
                                time_since_last_cmd.seconds());
        }
    }
    
    void stop() {
        left_motor_->stop();
        right_motor_->stop();
    }
    
    // Motor objects
    std::unique_ptr<Motor> left_motor_;
    std::unique_ptr<Motor> right_motor_;
    
    // ROS2 objects
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    
    // Parameters
    double track_width_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    
    // Safety tracking
    rclcpp::Time last_cmd_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RobotController>();
        
        // Set up signal handler for graceful shutdown
        rclcpp::on_shutdown([node]() {
            RCLCPP_INFO(node->get_logger(), "Shutting down robot controller...");
        });
        
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_controller"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}