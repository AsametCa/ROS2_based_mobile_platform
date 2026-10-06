/**
 * =============================================================================
 * @file odometry_node.cpp
 * @brief Differential drive odometry estimation node for Raspberry Pi.
 *
 * Responsibilities:
 * - Subscribes to wheel pulse velocities (/left_motor_vel, /right_motor_vel).
 * - Computes dead-reckoning position (x, y, theta) and velocities (linear, angular).
 * - Broadcasts standard nav_msgs/msg/Odometry messages on /odom.
 * - Broadcasts coordinate transform between odom and base_link frames.
 * =============================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <memory>

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() : Node("odometry_node") {
        // Declare node parameters with default dimensions
        this->declare_parameter("track_width",    0.30);    // Wheel-to-wheel separation [m]
        this->declare_parameter("wheel_diameter", 0.07);    // Wheel diameter [m]
        this->declare_parameter("counts_per_rev", 3200.0);  // Quadrature encoder pulses per revolution
        this->declare_parameter("publish_rate",   50.0);    // Odometry computation frequency [Hz]

        track_width_    = this->get_parameter("track_width").as_double();
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
        counts_per_rev_ = this->get_parameter("counts_per_rev").as_double();
        double rate     = this->get_parameter("publish_rate").as_double();

        // Distance traveled per single encoder count [m/count]
        m_per_count_ = (M_PI * wheel_diameter_) / counts_per_rev_;

        // Subscribe to motor velocity feedback from ESP32 (counts/sec)
        left_vel_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/left_motor_vel", 10,
            [this](const std_msgs::msg::Int32::SharedPtr m) { left_cps_ = m->data; });
        right_vel_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/right_motor_vel", 10,
            [this](const std_msgs::msg::Int32::SharedPtr m) { right_cps_ = m->data; });

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        double dt = 1.0 / rate;
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int64_t>(dt * 1e6)),
            std::bind(&OdometryNode::update, this));

        RCLCPP_INFO(this->get_logger(),
            "Odometry node started | track=%.2fm wheel=%.3fm cpr=%.0f",
            track_width_, wheel_diameter_, counts_per_rev_);
    }

private:
    void update() {
        auto now = this->get_clock()->now();
        if (!t_set_) {
            last_t_ = now;
            t_set_ = true;
            return;
        }

        double dt = (now - last_t_).seconds();
        last_t_ = now;
        if (dt <= 0.0 || dt > 0.5) return;

        // Convert pulse velocities to linear velocities [m/s]
        double v_left  = left_cps_  * m_per_count_;
        double v_right = right_cps_ * m_per_count_;

        // Forward kinematics for differential drive
        double v = (v_right + v_left) / 2.0;
        double w = (v_right - v_left) / track_width_;

        // Integrate pose using Euler forward integration
        theta_ += w * dt;
        x_ += v * std::cos(theta_) * dt;
        y_ += v * std::sin(theta_) * dt;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        // Broadcast odom -> base_link transform
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);

        // Publish nav_msgs/msg/Odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = w;

        // Covariance diagonal assignments
        odom.pose.covariance[0]   = 0.01; // X position variance
        odom.pose.covariance[7]   = 0.01; // Y position variance
        odom.pose.covariance[35]  = 0.03; // Yaw orientation variance
        odom.twist.covariance[0]  = 0.01; // Linear velocity variance
        odom.twist.covariance[35] = 0.03; // Angular velocity variance

        odom_pub_->publish(odom);
    }

    double track_width_, wheel_diameter_, counts_per_rev_, m_per_count_;
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    long left_cps_ = 0, right_cps_ = 0;
    rclcpp::Time last_t_;
    bool t_set_ = false;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_vel_sub_, right_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
