#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "robot/msg/laser_data.hpp"

using std::placeholders::_1;

class ReadingLaser : public rclcpp::Node {

public:
  ReadingLaser() : Node("reading_laser") {

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&ReadingLaser::topic_callback, this, _1));

      publisher_ = this->create_publisher<robot::msg::LaserData>("scan_data", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) const {

    robot::msg::LaserData laser_scan_output;

    for(auto & msg : _msg->ranges){
      RCLCPP_INFO(this->get_logger(), "Range : %f", msg);
      //laser_scan_output.data.push_back(static_cast<double>(msg));
    }

    laser_scan_output.header.stamp = _msg->header.stamp;
    laser_scan_output.ranges = std::vector<double>(_msg->ranges.begin(), _msg->ranges.end());
    publisher_->publish(laser_scan_output);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<robot::msg::LaserData>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReadingLaser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}