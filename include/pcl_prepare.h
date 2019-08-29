//#############################################################################
//
//	pcl_prepare.h
//
//#############################################################################
#ifndef __PCL_PREPARE_H__
#define __PCL_PREPARE_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

namespace mypcl
{

//#############################################################################
//
//  estimateNormals(): estimate normals of the input point cloud
//
//#############################################################################
int estimateNormals(const pcl::PointCloud<pcl::PointXYZ> 		&p_pcd,
					pcl::PointCloud<pcl::PointNormal> 			&pn_pcd);

//#############################################################################
//
//  estimateSIFT(): estimate SIFT keypoints of the input point cloud
//
//#############################################################################
int estimateSIFT(	const pcl::PointCloud<pcl::PointNormal> 	&pn_pcd,
					pcl::PointCloud<pcl::PointWithScale> 		&keypoints);

//#############################################################################
//
//  extractFPFH(): extract FPFH features from SIFT keypoints
//
//#############################################################################
int extractFPFH(const pcl::PointCloud<pcl::PointWithScale> 		&keypoints,
				pcl::PointCloud<pcl::FPFHSignature33> 			&features);

} //--- namespace mypcl

#endif //--- __PCL_PREPARE_H__