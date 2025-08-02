#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "robot/msg/velocity_data.hpp"

using std::placeholders::_1;

class Velocity : public rclcpp::Node {

public:
  Velocity() : Node("velocity") {

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Velocity::topic_callback, this, _1));

      publisher_ = this->create_publisher<robot::msg::VelocityData>("robot_velocity_data", 10);
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr _msg) const {

    double linear_vel = _msg->twist.twist.linear.x;
    double angular_vel = _msg->twist.twist.angular.z;

    RCLCPP_INFO(this->get_logger(), "Linear vel: %f\tAngular vel: %f", linear_vel, angular_vel);

    robot::msg::VelocityData robot_velocity_output;
    robot_velocity_output.linear = linear_vel;
    robot_velocity_output.angular = angular_vel;

    publisher_->publish(robot_velocity_output);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<robot::msg::VelocityData>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Velocity>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}