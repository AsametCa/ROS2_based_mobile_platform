/**
 * =============================================================================
 * @file pi_kinematics_node.cpp
 * @brief Differential drive inverse kinematics translator (Host PC / Pi side).
 *
 * Responsibilities:
 * - Subscribes to target velocity commands (/cmd_vel) from Nav2 or teleoperation.
 * - Decomposes linear and angular velocities into individual wheel pulse targets.
 * - Clamps outputs proportionally to preserve turn radius under velocity limits.
 * - Publishes setpoints to /left_motor_target and /right_motor_target.
 * =============================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <algorithm>
#include <cmath>

class DifferentialDriveLogic : public rclcpp::Node {
public:
    DifferentialDriveLogic() : Node("pi_kinematics_node") {
        // Declare node parameters with physical chassis attributes
        this->declare_parameter("track_width",    0.30);    // Wheel-to-wheel separation [m]
        this->declare_parameter("wheel_diameter", 0.07);    // Wheel diameter [m]
        this->declare_parameter("counts_per_rev", 3200.0);  // Quadrature encoder pulses per revolution
        this->declare_parameter("max_vel_cps",    6600.0);  // Maximum physical wheel velocity [counts/s]
        this->declare_parameter("target_ratio",   0.80);    // Safe operational ceiling ratio

        track_width_    = this->get_parameter("track_width").as_double();
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
        counts_per_rev_ = this->get_parameter("counts_per_rev").as_double();
        max_vel_cps_    = this->get_parameter("max_vel_cps").as_double();
        target_ratio_   = this->get_parameter("target_ratio").as_double();

        m_per_count_    = (M_PI * wheel_diameter_) / counts_per_rev_;
        max_target_cps_ = target_ratio_ * max_vel_cps_;

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DifferentialDriveLogic::cmdCallback, this, std::placeholders::_1));

        left_pub_  = this->create_publisher<std_msgs::msg::Int32>("/left_motor_target",  10);
        right_pub_ = this->create_publisher<std_msgs::msg::Int32>("/right_motor_target", 10);

        RCLCPP_INFO(this->get_logger(),
            "Kinematics node started | track=%.2fm wheel=%.3fm cpr=%.0f max_target=%.0f cps",
            track_width_, wheel_diameter_, counts_per_rev_, max_target_cps_);
    }

private:
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double v = msg->linear.x;   // Linear velocity [m/s]
        double w = msg->angular.z;  // Angular velocity [rad/s]

        // Differential drive inverse kinematics
        double v_right = v + w * (track_width_ * 0.5);
        double v_left  = v - w * (track_width_ * 0.5);

        // Convert linear wheel velocities to encoder counts per second
        double cps_right = v_right / m_per_count_;
        double cps_left  = v_left  / m_per_count_;

        // Proportional velocity scaling to maintain turning radius when exceeding limits
        double m = std::max(std::fabs(cps_right), std::fabs(cps_left));
        if (m > max_target_cps_) {
            double scale = max_target_cps_ / m;
            cps_right *= scale;
            cps_left  *= scale;
        }

        auto right_msg = std_msgs::msg::Int32();
        auto left_msg  = std_msgs::msg::Int32();
        right_msg.data = static_cast<int32_t>(std::round(cps_right));
        left_msg.data  = static_cast<int32_t>(std::round(cps_left));

        right_pub_->publish(right_msg);
        left_pub_->publish(left_msg);
    }

    double track_width_, wheel_diameter_, counts_per_rev_;
    double max_vel_cps_, target_ratio_, m_per_count_, max_target_cps_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveLogic>());
    rclcpp::shutdown();
    return 0;
}
