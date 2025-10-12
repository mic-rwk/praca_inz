#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "robot/msg/laser_data.hpp"

using std::placeholders::_1;

class DiffLaser : public rclcpp::Node
{
public:
    DiffLaser() : Node("diff_laser")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DiffLaser::topic_callback, this, _1));

        publisher_ = this->create_publisher<robot::msg::LaserData>("scan_diff", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        if (!_msg || _msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty LaserScan message.");
            return;
        }

        if (!prev_scan_) {
            prev_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*_msg);
            return;
        }

        if (prev_scan_->ranges.size() != _msg->ranges.size()) {
            prev_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*_msg);
            return;
        }

        robot::msg::LaserData diff_msg;
        diff_msg.header.stamp = _msg->header.stamp;
        diff_msg.ranges.resize(_msg->ranges.size());

        for (size_t i = 0; i < _msg->ranges.size(); ++i) {
            float current = _msg->ranges[i];
            float previous = prev_scan_->ranges[i];

            if (std::isnan(current) || std::isnan(previous)) {
                diff_msg.ranges[i] = std::numeric_limits<double>::quiet_NaN();
            } 
            else if(std::isinf(current) || std::isinf(previous)) {
                diff_msg.ranges[i] = std::numeric_limits<double>::infinity();
            }
            else {
                diff_msg.ranges[i] = static_cast<double>(current - previous);
            }
        }

        publisher_->publish(diff_msg);

        *prev_scan_ = *_msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<robot::msg::LaserData>::SharedPtr publisher_;
    std::shared_ptr<sensor_msgs::msg::LaserScan> prev_scan_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffLaser>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
