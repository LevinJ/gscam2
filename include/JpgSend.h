/*
 * JpgSend.h
 *
 *  Created on: Mar 9, 2023
 *      Author: levin
 */

#ifndef JPGSEND_H_
#define JPGSEND_H_
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include "TimerUtil.h"
#include <cmath>
namespace semantic_slam {

class JpgSend {
public:
	JpgSend(rclcpp::Node * pnode, std::string pub_topic_name){ 
		jpeg_pub_ =  pnode->create_publisher<sensor_msgs::msg::CompressedImage>(pub_topic_name, 1);
	}
	virtual ~JpgSend(){}
	void pub_jpg(cv::Mat mat, std_msgs::msg::Header & header, double resize_ratio = 1.0){
		if(std::abs(resize_ratio - 1) > 1e-6){
			cv::resize(mat, mat, cv::Size(), resize_ratio, resize_ratio);
		} 
		auto img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        //compress the image
        // semantic_slam::TimerUtil timer;
        std::vector<uchar> &buff = img->data;//buffer for coding
        std::vector<int> param(2);
        param[0] = cv::IMWRITE_JPEG_QUALITY;
        param[1] = 95;//default(95) 0-100
        cv::imencode(".jpg", mat, buff, param);

        
    	img->header = header;
        img->format = "jpeg";
        // std::cout<<"encode time="<<timer.elapsed()<<std::endl;
        // timer.reset();
        jpeg_pub_->publish(std::move(img));

        // std::cout<<"publish time 1="<<timer.elapsed()<<std::endl;

	}
private:
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub_;
};

} /* namespace semantic_slam */

#endif /* JPGSEND_H_ */
