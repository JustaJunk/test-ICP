//#############################################################################
//
//	pcl_register.h
//
//#############################################################################
#ifndef __PCL_REGISTER_H__
#define __PCL_REGISTER_H__

#include "pcl_prepare.h"

namespace mypcl
{

//#############################################################################
//
//  featureAlign(): align point cloud by features
//
//#############################################################################
int featureAlign(	const scalar_cloud		&dst_keypoints,
					const feature_cloud 	&dst_features,
					const scalar_cloud 		&src_keypoints,
					const feature_cloud 	&src_features,
					Eigen::Matrix4f 		transformation);

//#############################################################################
//
//  roughAlign(): roughly align 2 point cloud
//
//#############################################################################
int roughAlign(		const point_cloud		&dst_points,
					const point_cloud 		&src_points,
					Eigen::Matrix4f 		transformation);

//#############################################################################
//
//  preciseAlign(): precisely align 2 point cloud
//	
//#############################################################################
int preciseAlign(	const point_cloud		&dst_points,
					const point_cloud 		&src_points,
					Eigen::Matrix4f 		transformation);

//#############################################################################
//
//  hybridAlign(): roughly align and then precisely align
//	
//#############################################################################
int hybridAlign(	const point_cloud		&dst_points,
					const point_cloud 		&src_points,
					Eigen::Matrix4f 		transformation);

} //--- namespace mypcl

#endif //--- __PCL_REGISTER_H__