#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <chrono>
#include <sstream>
#include "motor.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        // Declare parameters
        this->declare_parameter<int>("left_motor", 0);
        this->declare_parameter<int>("right_motor", 1);
        this->declare_parameter<double>("left_motor_alpha", 1.0);
        this->declare_parameter<double>("right_motor_alpha", 1.0);
        this->declare_parameter<double>("left_motor_beta", 0.0);   // Add beta parameters
        this->declare_parameter<double>("right_motor_beta", 0.0);
        this->declare_parameter<double>("track_width", 0.11);
        this->declare_parameter<double>("max_linear_velocity", 1.0);
        this->declare_parameter<double>("max_angular_velocity", 2.0);

        // Get parameters
        int left_motor = this->get_parameter("left_motor").as_int();
        int right_motor = this->get_parameter("right_motor").as_int();
        double left_alpha = this->get_parameter("left_motor_alpha").as_double();
        double right_alpha = this->get_parameter("right_motor_alpha").as_double();
        double left_beta = this->get_parameter("left_motor_beta").as_double();
        double right_beta = this->get_parameter("right_motor_beta").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();

        left_motor_ = std::make_unique<Motor>(this->get_logger(), left_motor, left_alpha, left_beta);
        right_motor_ = std::make_unique<Motor>(this->get_logger(), right_motor, right_alpha, right_beta);

        stop();

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            10, 
            std::bind(&RobotController::cmdVelCallback, this, std::placeholders::_1)
        );

        status_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotController::publishStatus, this)
        );

        safety_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RobotController::safetyCheck, this)
        );

        last_cmd_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Robot controller initialized");
    }

    ~RobotController() {
        stop();
        RCLCPP_INFO(this->get_logger(), "Robot controller shutting down");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = this->get_clock()->now();

        double lin_vel = msg->linear.x;
        double ang_vel = msg->angular.z;

        lin_vel = std::max(-max_linear_velocity_, std::min(max_linear_velocity_, lin_vel));
        ang_vel = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, ang_vel));

        double left_vel = lin_vel - (ang_vel * track_width_ / 2.0);
        double right_vel = lin_vel + (ang_vel * track_width_ / 2.0);

        double max_wheel_vel = std::max(std::abs(left_vel), std::abs(right_vel));
        if (max_wheel_vel > max_linear_velocity_) {
            left_vel = left_vel / max_wheel_vel * max_linear_velocity_;
            right_vel = right_vel / max_wheel_vel * max_linear_velocity_;
        }

        left_vel = std::clamp(left_vel / max_linear_velocity_, -1.0, 1.0);
        right_vel = std::clamp(right_vel / max_linear_velocity_, -1.0, 1.0);

        left_motor_->setValue(left_vel);
        right_motor_->setValue(right_vel);

        last_left_value_ = left_vel;
        last_right_value_ = right_vel;
    }

    void publishStatus() {
        auto message = std_msgs::msg::String();
        std::ostringstream ss;
        ss << "Robot Status. Left=" << last_left_value_
           << " Right=" << last_right_value_
           << " Standby: Active";
        message.data = ss.str();
        status_publisher_->publish(message);
    }

    void safetyCheck() {
        auto now = this->get_clock()->now();
        auto time_since_last_cmd = now - last_cmd_time_;

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
        last_left_value_ = 0.0;
        last_right_value_ = 0.0;
    }

    std::unique_ptr<Motor> left_motor_;
    std::unique_ptr<Motor> right_motor_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;

    double track_width_;
    double max_linear_velocity_;
    double max_angular_velocity_;

    double last_left_value_ = 0.0;
    double last_right_value_ = 0.0;

    rclcpp::Time last_cmd_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<RobotController>();

        rclcpp::on_shutdown([node]() {
            RCLCPP_INFO(node->get_logger(), "Shutting down JetTank robot controller...");
        });

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_controller"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
