#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class Encoder : public rclcpp::Node {

public:
  Encoder() : Node("encoder") {

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Encoder::topic_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("encoder_data", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr _msg) const {

    double left_velocity = 0.0;
    double right_velocity = 0.0;

    for (size_t i = 0; i < _msg->velocity.size(); i++) {
      if (_msg->name[i] == "left_wheel_joint") {
        left_velocity = _msg->velocity[i];
      } else if (_msg->name[i] == "right_wheel_joint") {
        right_velocity = _msg->velocity[i];
      }
    }

    RCLCPP_INFO(this->get_logger(), "Left vel: %f\tRight vel: %f", left_velocity, right_velocity);
    
    std_msgs::msg::Float64MultiArray encoder_output;
    encoder_output.data.push_back(left_velocity);
    encoder_output.data.push_back(right_velocity);

    publisher_->publish(encoder_output);
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Encoder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}