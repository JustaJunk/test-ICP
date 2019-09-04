//#############################################################################
//
//	cv_prepare.h
//
//#############################################################################
#ifndef __CV_PREPARE_H__
#define __CV_PREPARE_H__

#include <opencv2/core.hpp>

namespace mycv
{

//#############################################################################
//
//  findCloudBox(): find the standard bonding box of the input point cloud
//
//#############################################################################
int findCloudBox(	const cv::Mat 		&point_cloud,
					cv::Vec2f 			&xRange,
					cv::Vec2f 			&yRange,
					cv::Vec2f 			&zRange);

//#############################################################################
//
//  sampleCloudUniform(): down sample the input point cloud
//
//#############################################################################
int sampleCloudUniform(	const cv::Mat 		&point_cloud,
						const int 			&sample_step,
						cv::Mat 			&sample_cloud);

//#############################################################################
//
//  sampleCloudGridstep(): down sample the input point cloud
//
//#############################################################################
int sampleCloudGridstep(const cv::Mat 		&point_cloud,
						const float 		&grid_step,
						cv::Mat 			&sample_cloud);

} //--- namespace mycv

#endif //--- __CV_PREPARE_H__