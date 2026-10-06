/**
 * =============================================================================
 * @file trajectory_publisher.cpp
 * @brief Robot trajectory accumulator and drift comparison publisher.
 *
 * Responsibilities:
 * - Listens to odom -> base_link transform and publishes /odom_path (accumulated raw odometry).
 * - Listens to map -> base_link transform and publishes /slam_path (SLAM/AMCL corrected trajectory).
 * - Enforces minimum distance thresholds between path poses to conserve memory.
 * =============================================================================
 */

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        // Declare coordinate frame names and update parameters
        map_frame_  = this->declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        double rate = this->declare_parameter<double>("publish_rate", 10.0); // Path update frequency [Hz]
        min_dist_   = this->declare_parameter<double>("min_dist", 0.02);     // Minimum step displacement [m]

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Path>("/odom_path", 10);
        slam_pub_ = this->create_publisher<nav_msgs::msg::Path>("/slam_path", 10);

        odom_path_.header.frame_id = odom_frame_;
        slam_path_.header.frame_id = map_frame_;

        have_last_odom_ = false;
        have_last_slam_ = false;

        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TrajectoryPublisher::onTimer, this));

        RCLCPP_INFO(this->get_logger(),
            "Trajectory publisher started. Publishing: /odom_path (%s->%s), /slam_path (%s->%s)",
            odom_frame_.c_str(), base_frame_.c_str(),
            map_frame_.c_str(), base_frame_.c_str());
    }

private:
    bool lookupXYYaw(const std::string &target, const std::string &source,
                     double &x, double &y, double &yaw) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            // tf2::TimePointZero queries the latest available transform in the buffer
            tf = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
        } catch (const tf2::TransformException &) {
            return false;
        }
        x = tf.transform.translation.x;
        y = tf.transform.translation.y;
        const auto &q = tf.transform.rotation;
        yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return true;
    }

    geometry_msgs::msg::PoseStamped makePose(
        const std::string &frame_id, const rclcpp::Time &stamp,
        double x, double y, double yaw) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id;
        ps.header.stamp = stamp;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0.0;
        ps.pose.orientation.z = std::sin(yaw / 2.0);
        ps.pose.orientation.w = std::cos(yaw / 2.0);
        return ps;
    }

    bool farEnough(bool have_last, double lx, double ly, double x, double y) {
        if (!have_last) return true;
        double dx = x - lx, dy = y - ly;
        return (dx * dx + dy * dy) >= (min_dist_ * min_dist_);
    }

    void onTimer() {
        rclcpp::Time now = this->get_clock()->now();
        double x, y, yaw;

        // 1. Process raw odometry trajectory (odom -> base_link)
        if (lookupXYYaw(odom_frame_, base_frame_, x, y, yaw)) {
            if (farEnough(have_last_odom_, last_odom_x_, last_odom_y_, x, y)) {
                odom_path_.poses.push_back(makePose(odom_frame_, now, x, y, yaw));
                last_odom_x_ = x;
                last_odom_y_ = y;
                have_last_odom_ = true;
            }
            odom_path_.header.stamp = now;
            odom_pub_->publish(odom_path_);
        }

        // 2. Process SLAM/AMCL corrected trajectory (map -> base_link)
        if (lookupXYYaw(map_frame_, base_frame_, x, y, yaw)) {
            if (farEnough(have_last_slam_, last_slam_x_, last_slam_y_, x, y)) {
                slam_path_.poses.push_back(makePose(map_frame_, now, x, y, yaw));
                last_slam_x_ = x;
                last_slam_y_ = y;
                have_last_slam_ = true;
            }
            slam_path_.header.stamp = now;
            slam_pub_->publish(slam_path_);
        }
    }

    std::string map_frame_, odom_frame_, base_frame_;
    double min_dist_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_pub_;

    nav_msgs::msg::Path odom_path_;
    nav_msgs::msg::Path slam_path_;

    bool have_last_odom_, have_last_slam_;
    double last_odom_x_ = 0.0, last_odom_y_ = 0.0;
    double last_slam_x_ = 0.0, last_slam_y_ = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
