#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

class ReadingLaser : public rclcpp::Node {

public:
  ReadingLaser() : Node("reading_laser") {

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&ReadingLaser::topic_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("scan_data", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) const {

    std_msgs::msg::Float64MultiArray laser_scan_output;

    for(auto & msg : _msg->ranges){
      RCLCPP_INFO(this->get_logger(), "Range : %f", msg);
      laser_scan_output.data.push_back(static_cast<double>(msg));
    }
    publisher_->publish(laser_scan_output);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReadingLaser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}