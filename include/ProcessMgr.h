/*
 * ProcessMgr.h
 *
 *  Created on: May 22, 2023
 *      Author: levin
 */

#ifndef PROCESSMGR_H_
#define PROCESSMGR_H_

#include <opencv2/imgproc.hpp>

namespace semantic_slam {

class ProcessMgr {
public:
	ProcessMgr();
	virtual ~ProcessMgr();

	void process_img(cv::Mat im);

	cv::Mat undistored_ocam;
	cv::Mat undistored_fisheye;
private:
	void undistort_ocam(cv::Mat im);


};

} /* namespace semantic_slam */

#endif /* PROCESSMGR_H_ */
