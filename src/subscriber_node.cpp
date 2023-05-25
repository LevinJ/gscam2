#include "gscam2/subscriber_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace gscam2
{

ImageSubscriberNode::ImageSubscriberNode(const rclcpp::NodeOptions & options)
: Node("image_subscriber", options)
{
  ocam_pub_ = this->create_publisher<sensor_msgs::msg::Image>("undistorted_ocam", 1);
  fisheye_pub_ = this->create_publisher<sensor_msgs::msg::Image>("undistorted_fisheye", 1);
  (void) sub_;

  RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());

  sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 1,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
      RCLCPP_INFO_ONCE(get_logger(), "receiving messages");    // NOLINT

      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      cv::Mat img = cv_ptr->image;
      //crop the image
      img = img(cv::Rect(0,0,1920,1080));

      // std::string img_path = "/media/levin/DATA/zf/semantic_seg/20230511/in_jt_campus/1680156423306309888.jpg";
      // img = cv::imread(img_path);
      pm_.process_img(img);

      // sensor_msgs::msg::Image::SharedPtr tmsg = ;
      ocam_pub_->publish(*(cv_bridge::CvImage(cv_ptr->header, "bgr8", pm_.undistored_ocam).toImageMsg()));
      fisheye_pub_->publish(*(cv_bridge::CvImage(cv_ptr->header, "bgr8", pm_.undistored_fisheye).toImageMsg()));

      cv::imshow("original", img);
      cv::imshow("fisheye undistorted", pm_.undistored_fisheye);
      cv::imshow("ocam undistorted", pm_.undistored_ocam);
      cv::waitKey(1);    // Look for key presses.

// #undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
      static int count = 0;
      RCLCPP_INFO(get_logger(), "%d, %p", count++, (void *)reinterpret_cast<std::uintptr_t>(msg.get()));
#else
      (void) this;
      (void) msg;
#endif
    });

  RCLCPP_INFO(get_logger(), "ready");
}

} // namespace gscam2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam2::ImageSubscriberNode)  // NOLINT
