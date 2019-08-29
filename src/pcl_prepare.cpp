//#############################################################################
//
//	pcl_prepare.cpp
//
//#############################################################################
#include <cstdlib>
#include <iostream>
#include "pcl_prepare.h"

namespace mypcl
{

//#############################################################################
//
//  estimateNormals(): estimate normals of the input point cloud
//
//#############################################################################
int estimateNormals(const pcl::PointCloud<pcl::PointXYZ> 		&p_pcd,
					pcl::PointCloud<pcl::PointNormal> 			&pn_pcd)
{
	return EXIT_SUCCESS;
}

//#############################################################################
//
//  estimateSIFT(): estimate SIFT keypoints of the input point cloud
//
//#############################################################################
int estimateSIFT(	const pcl::PointCloud<pcl::PointNormal> 	&pn_pcd,
					pcl::PointCloud<pcl::PointWithScale> 		&keypoints)
{
	return EXIT_SUCCESS;
}

//#############################################################################
//
//  extractFPFH(): extract FPFH features from SIFT keypoints
//
//#############################################################################
int extractFPFH(const pcl::PointCloud<pcl::PointWithScale> 		&keypoints,
				pcl::PointCloud<pcl::FPFHSignature33> 			&features)
{
	return EXIT_SUCCESS;
}

} //--- namespace mypcl