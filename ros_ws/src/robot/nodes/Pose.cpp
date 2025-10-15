#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include "robot/msg/pose_data.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

using std::placeholders::_1;

class Pose : public rclcpp::Node {

public:
  Pose() : Node("pose") {

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", 10, std::bind(&Pose::topic_callback, this, _1));

      publisher_ = this->create_publisher<robot::msg::PoseData>("pose_data", 10);
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr _msg) const {

    robot::msg::PoseData pose_output;
    pose_output.header.stamp = _msg->header.stamp;
    pose_output.x_pose = _msg->pose.pose.position.x;
    pose_output.y_pose = _msg->pose.pose.position.y;

    auto x = _msg->pose.pose.orientation.x;
    auto y = _msg->pose.pose.orientation.y;
    auto z = _msg->pose.pose.orientation.z;
    auto w = _msg->pose.pose.orientation.w;
    tf2::Quaternion quaternion(x, y, z, w);
    double yaw = tf2::getYaw(quaternion);
    pose_output.yaw_pose = yaw;

    RCLCPP_INFO(this->get_logger(), "Pose - X: %f\tY: %f\tYaw: %f", pose_output.x_pose, pose_output.y_pose, pose_output.yaw_pose);

    publisher_->publish(pose_output);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Publisher<robot::msg::PoseData>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}