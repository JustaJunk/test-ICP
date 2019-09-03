//#############################################################################
//
//	cv_prepare.cpp
//
//#############################################################################
#include <cstdlib>
#include <iostream>
#include "cv_prepare.h"

namespace mycv
{

//#############################################################################
//
//  sampleUniformCloud(): down sample the input point cloud
//
//#############################################################################
int sampleUniformCloud(	const cv::Mat 		&point_cloud,
						const int 			&sample_step,
						cv::Mat 			&sample_cloud)
{
	int point_count 	= point_cloud.rows;
	int sample_count 	= point_count/sample_step;
	int sample_idx 		= 0;

	sample_cloud.release();
	sample_cloud.create(sample_count, point_cloud.cols, point_cloud.type());

	for (int idx = 0; idx < point_count && sample_idx < sample_count; idx += sample_step)
	{
		point_cloud.row(idx).copyTo(sample_cloud.row(sample_idx++));
	}

	return EXIT_SUCCESS;
}

//#############################################################################
//
//  findCloudBox(): find the standard bonding box of the input point cloud
//
//#############################################################################
int findCloudBox(	const cv::Mat 		&point_cloud,
					cv::Vec2f 			&xRange,
					cv::Vec2f 			&yRange,
					cv::Vec2f 			&zRange)
{
	int 			count 	= point_cloud.rows;
	const float* 	first	= point_cloud.ptr<float>(0);
	float 			x, y, z;

	xRange[0] = first[0];
	xRange[1] = first[0];
	yRange[0] = first[1];
	yRange[1] = first[1];
	zRange[0] = first[2];
	zRange[1] = first[2];

	for (int i = 1; i < count; ++i)
	{
		const float* points = point_cloud.ptr<float>(i);
		x = points[0];
		y = points[1];
		z = points[2];
		if (x < xRange[0])	xRange[0] = x;
		if (x > xRange[1])	xRange[1] = x;
		if (y < yRange[0])	yRange[0] = y;
		if (y > yRange[1])	yRange[1] = y;
		if (z < zRange[0])	zRange[0] = z;
		if (z > zRange[1])	zRange[1] = z;
	}	

	return EXIT_SUCCESS;
}

} //--- namespace mycv