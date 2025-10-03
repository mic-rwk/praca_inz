#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "robot/msg/collected_data.hpp"
#include "robot/msg/encoder_data.hpp"
#include "robot/msg/laser_data.hpp"
#include "robot/msg/velocity_data.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<robot::msg::EncoderData,
                                                                         robot::msg::VelocityData,
                                                                         robot::msg::LaserData,
                                                                         robot::msg::LaserData>;

class RobotMonitor : public rclcpp::Node {
public:
  RobotMonitor() : Node("robot_monitor") {

    encoder_subscription.subscribe(this, "encoder_data");
    velocity_subscription.subscribe(this, "robot_velocity_data");
    laser_subscription.subscribe(this, "scan_data");
    diff_laser_subscription.subscribe(this, "scan_diff");

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>
    (ApproxSyncPolicy(10), encoder_subscription, velocity_subscription, laser_subscription, diff_laser_subscription);

    sync_->registerCallback(std::bind(&RobotMonitor::synchronized_callback, this, _1, _2, _3, _4));

    publisher_ = this->create_publisher<robot::msg::CollectedData>("robot_monitor", 10);
  }

private:
  void synchronized_callback(const robot::msg::EncoderData::ConstSharedPtr encoder_msg,
                             const robot::msg::VelocityData::ConstSharedPtr velocity_msg,
                             const robot::msg::LaserData::ConstSharedPtr scan_msg,
                             const robot::msg::LaserData::ConstSharedPtr diff_scan_msg) {

    RCLCPP_INFO(this->get_logger(), "Encoder: Left: %f Right: %f", encoder_msg->left, encoder_msg->right);

    RCLCPP_INFO(this->get_logger(), "Velocity: Linear: %f Angular: %f", velocity_msg->linear, velocity_msg->angular);

    RCLCPP_INFO(this->get_logger(), "Laser ranges size: %d:", scan_msg->ranges.size());
    for (size_t i = 0; i < 5; i++) {
      RCLCPP_INFO(this->get_logger(), " %f", scan_msg->ranges[i]);
    }

    robot::msg::CollectedData robot_monitor_output;
    robot_monitor_output.encoder_data = *encoder_msg;
    robot_monitor_output.velocity_data = *velocity_msg;
    robot_monitor_output.laser_data = *scan_msg;
    robot_monitor_output.diff_laser_data = *diff_scan_msg;
    robot_monitor_output.header.stamp = this->get_clock()->now();

    publisher_->publish(robot_monitor_output);
  }

  message_filters::Subscriber<robot::msg::EncoderData> encoder_subscription;
  message_filters::Subscriber<robot::msg::VelocityData> velocity_subscription;
  message_filters::Subscriber<robot::msg::LaserData> laser_subscription;
  message_filters::Subscriber<robot::msg::LaserData> diff_laser_subscription;

  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

  rclcpp::Publisher<robot::msg::CollectedData>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}