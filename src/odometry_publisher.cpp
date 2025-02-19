#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node {
public:
    OdometryPublisher()
        : Node("odometry_publisher"), x_(0.0), y_(0.0), th_(0.0), ticks_per_meter_(4900), wheel_base_(0.72) {

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/encoder_data", 10,
            std::bind(&OdometryPublisher::encoder_callback, this, std::placeholders::_1));

        last_time_ = this->get_clock()->now();
        last_ticks_ = {0, 0, 0, 0, 0, 0};
    }

private:
    void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Encoder message does not have enough data.");
            return;
        }

        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        if (dt == 0) return;

        std::vector<int> current_ticks = msg->data;
        double right_distance = ((current_ticks[0] - last_ticks_[0]) + (current_ticks[2] - last_ticks_[2]) ) / (2.0 * ticks_per_meter_);
        double left_distance = ((current_ticks[1] - last_ticks_[1]) + (current_ticks[3] - last_ticks_[3])  )/ (2.0 * ticks_per_meter_);

        last_ticks_ = current_ticks;
        
        double distance = (left_distance + right_distance) / 2.0;
        double delta_theta = (right_distance - left_distance) / wheel_base_;

        x_ += distance * std::cos(th_ + delta_theta / 2.0);
        y_ += distance * std::sin(th_ + delta_theta / 2.0);
        th_ += delta_theta;

        publish_odometry(current_time, distance / dt, delta_theta / dt);
        last_time_ = current_time;
    }

    void publish_odometry(const rclcpp::Time &current_time, double linear_velocity, double angular_velocity) {
        geometry_msgs::msg::Quaternion odom_quat;
        odom_quat.x = 0.0;
        odom_quat.y = 0.0;
        odom_quat.z = std::sin(th_ / 2);
        odom_quat.w = std::cos(th_ / 2);

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        tf_broadcaster_->sendTransform(odom_trans);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.angular.z = angular_velocity;
        odom_pub_->publish(odom);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Time last_time_;
    std::vector<int> last_ticks_;
    double x_, y_, th_;
    const double ticks_per_meter_;
    const double wheel_base_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}