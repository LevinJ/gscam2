/*
 * ProcessMgr.cpp
 *
 *  Created on: May 22, 2023
 *      Author: levin
 */

#include "ProcessMgr.h"
#include "bev/image_stitch.h"
#include "opencv2/calib3d/calib3d.hpp"

namespace semantic_slam
{

	ProcessMgr::ProcessMgr()
	{
		// TODO Auto-generated constructor stub
	}

	ProcessMgr::~ProcessMgr()
	{
		// TODO Auto-generated destructor stub
	}
	void ProcessMgr::process_img(cv::Mat im)
	{
		undistored_fisheye = undistort_fisheye(im);
		undistort_ocam(im);
	}
	cv::Mat ProcessMgr::undistort_fisheye(cv::Mat src)
	{
		cv::Mat cameraMatrix_ = cv::Mat::eye(3, 3, CV_64F);
		cameraMatrix_.at<double>(0, 0) = 508.8168476420729;
		cameraMatrix_.at<double>(0, 2) = 978.64245209484579;
		cameraMatrix_.at<double>(1, 1) = 508.73048398231685;
		cameraMatrix_.at<double>(1, 2) = 537.32130468796674;

		cv::Mat distCoeffs_ = cv::Mat::zeros(4, 1, CV_64F);
		distCoeffs_.at<double>(0, 0) = 0.14118276716783112;
		distCoeffs_.at<double>(1, 0) = -0.046538334132839872;
		distCoeffs_.at<double>(2, 0) = 0.0054513296832681178;
		distCoeffs_.at<double>(3, 0) = -0.00050583284890585566;

		cv::Size imageSize = src.size();
		cv::Mat dst_persp = cv::Mat::zeros(imageSize, CV_8UC3);
		static cv::Mat mapx_persp = cv::Mat::zeros(src.rows, src.cols, CV_32FC1);
		static cv::Mat mapy_persp = cv::Mat::zeros(src.rows, src.cols, CV_32FC1);
		static bool first_time = true;
		static cv::Mat newIntMat_;
		if (first_time)
		{
			first_time = false;
			cv::Mat newIntMat_ = cv::Mat::eye(3, 3, CV_64F);
			newIntMat_.at<double>(0, 0) = 400.9387932865325;
			newIntMat_.at<double>(0, 2) = 959.5;
			newIntMat_.at<double>(1, 1) = 400.8603864394393;
			newIntMat_.at<double>(1, 2) = 539.5;
			// cv::Mat newIntMat_ = cv::getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_, imageSize, 0, imageSize, 0, true);

			//    std::cout<<"undistored img: "<<img<<std::endl;

			cv::fisheye::initUndistortRectifyMap(cameraMatrix_, distCoeffs_, cv::Matx33d::eye(), newIntMat_, imageSize, CV_16SC2, mapx_persp, mapy_persp);
		}

		cv::remap(src, dst_persp, mapx_persp, mapy_persp, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0)); // correct the distortio

		return dst_persp;
	}
	void ProcessMgr::undistort_ocam(cv::Mat im)
	{
		std::string intrinsics_path = "/home/levin/workspace/driver_vslam_ros2/install/fisheye_stitching_ros2/share/fisheye_stitching_ros2/config/intrinsics";
		// std::string intrinsics_path = "/home/levin/workspace/driver_vslam_ros2/install/fisheye_stitching_ros2/share/fisheye_stitching_ros2/config/intrinsics";
		ImageStitch obj;

		std::string intrinsic_path = intrinsics_path + "/fisheye_front.yaml";
		float sf = 5;

		// test_fisheye_model(img_path);
		undistored_ocam = obj.gen_undist2(im, intrinsic_path, sf);
	}
} /* namespace semantic_slam */
