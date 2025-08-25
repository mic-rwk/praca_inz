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

class DiffLaser : public rclcpp::Node
{
public:
    DiffLaser() : Node("diff_laser")
    {
        subscription_ = this->create_subscription<robot::msg::LaserData>(
            "/scan_data", 10, std::bind(&DiffLaser::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<robot::msg::LaserData>("/scan_diff", 10);
    }

private:
    void topic_callback(const robot::msg::LaserData::SharedPtr _msg)
    {
        if (!prev_scan_) {
            prev_scan_ = std::make_shared<robot::msg::LaserData>();
            prev_scan_->header.stamp = _msg->header.stamp;
            prev_scan_->ranges = std::vector<double>(_msg->ranges.begin(), _msg->ranges.end());
            return;
        }

        robot::msg::LaserData::SharedPtr diff_msg = std::make_shared<robot::msg::LaserData>();
        diff_msg->header.stamp = _msg->header.stamp;
        diff_msg->ranges = std::vector<double>(_msg->ranges.begin(), _msg->ranges.end());

        for(size_t i = 0; i < _msg->ranges.size(); i++)
        {
            diff_msg->ranges[i] = _msg->ranges[i] - prev_scan_->ranges[i];
            RCLCPP_INFO(this->get_logger(), "Range : %f", diff_msg->ranges[i]);
        }

        publisher_->publish(*diff_msg);
        prev_scan_->header.stamp = _msg->header.stamp;
        prev_scan_->ranges = std::vector<double>(_msg->ranges.begin(), _msg->ranges.end());
    }

    rclcpp::Subscription<robot::msg::LaserData>::SharedPtr subscription_;
    rclcpp::Publisher<robot::msg::LaserData>::SharedPtr publisher_;
    robot::msg::LaserData::SharedPtr prev_scan_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffLaser>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
