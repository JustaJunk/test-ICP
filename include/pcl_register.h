//#############################################################################
//
//	pcl_register.h
//
//#############################################################################
#ifndef __PCL_REGISTER_H__
#define __PCL_REGISTER_H__

#include "pcl_prepare.h"

//using point_cloud 	= pcl::PointCloud<pcl::PointXYZ>;
//using normal_cloud 	= pcl::PointCloud<pcl::PointNormal>;
//using scalar_cloud 	= pcl::PointCloud<pcl::PointWithScale>;
//using feature_cloud	= pcl::PointCloud<pcl::FPFHSignature33>;

namespace mypcl
{

//#############################################################################
//
//  roughAlign(): roughly align 2 point cloud
//
//#############################################################################
int roughAlign(		const scalar_cloud		&dst_keypoints,
					const feature_cloud 	&dst_features,
					const scalar_cloud 		&src_keypoints,
					const feature_cloud 	&src_features,
					Eigen::Matrix4f 		transformation);

//#############################################################################
//
//  preciseAlign(): precisely align 2 point cloud
//	
//#############################################################################
int preciseAlign(	const point_cloud		&dst_pc,
					const point_cloud 		&src_pc,
					Eigen::Matrix4f 		transformation);

} //--- namespace mypcl

#endif //--- __PCL_REGISTER_H__