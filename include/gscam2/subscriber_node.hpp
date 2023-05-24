#ifndef SIMPLE_SUBSCRIBER_HPP
#define SIMPLE_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ProcessMgr.h"

namespace gscam2
{

// Node that subscribes to a topic, used for testing composition and IPC
class ImageSubscriberNode : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

public:
  explicit ImageSubscriberNode(const rclcpp::NodeOptions & options);
private:
  // Publish images...
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ocam_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fisheye_pub_;
};

semantic_slam::ProcessMgr pm_;

} // namespace gscam2

#endif //SIMPLE_SUBSCRIBER_HPP
