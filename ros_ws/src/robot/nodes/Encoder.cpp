#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class Encoder : public rclcpp::Node {

public:
  Encoder() : Node("encoder") {

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Encoder::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr _msg) const {

    double left_vel = 0.0;
    double right_vel = 0.0;

    for (size_t i = 0; i < _msg->velocity.size(); i++) {
      
      if (_msg->name[i] == "left_wheel_joint") {
        left_vel = _msg->velocity[i];
      } else if (_msg->name[i] == "right_wheel_joint") {
        right_vel = _msg->velocity[i];
      }
    }

    RCLCPP_INFO(this->get_logger(), "Left vel: %f\tRight vel: %f", left_vel, right_vel);
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Encoder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}