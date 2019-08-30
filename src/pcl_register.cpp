//#############################################################################
//
//	pcl_register.cpp
//
//#############################################################################
#include <cstdlib>
#include <iostream>
#include "pcl_register.h"

namespace mypcl
{

//#############################################################################
//
//  roughAlign(): roughly align 2 point cloud with local feature
//
//#############################################################################
int roughAlign(		const scalar_cloud		&dst_keypoints,
					const feature_cloud 	&dst_features,
					const scalar_cloud 		&src_keypoints,
					const feature_cloud 	&src_features,
					Eigen::Matrix4f 		transformation)
{
	//--- Parameters
	const float min_sample_distance 		= 0.025f;
	const float max_correspondence_distance = 0.01f;
	const int 	nr_iterations 				= 500;

	//--- Setup aligner
	pcl::SampleConsensusInitialAlignment<	pcl::PointWithScale, 
											pcl::PointWithScale, 
											pcl::FPFHSignature33> 	ra;
	ra.setMinSampleDistance(min_sample_distance);
	ra.setMaxCorrespondenceDistance(max_correspondence_distance);
	ra.setMaximumIterations(nr_iterations);

	//--- Setup input
	scalar_cloud::Ptr  	dst_keypoints_ptr(new scalar_cloud);
	feature_cloud::Ptr 	dst_features_ptr(new feature_cloud);
	scalar_cloud::Ptr 	src_keypoints_ptr(new scalar_cloud);
	feature_cloud::Ptr 	src_features_ptr(new feature_cloud);
	*dst_keypoints_ptr	= dst_keypoints;
	*dst_features_ptr	= dst_features;
	*src_keypoints_ptr	= src_keypoints;
	*src_features_ptr	= src_features;
	ra.setInputTarget(dst_keypoints_ptr);
	ra.setTargetFeatures(dst_features_ptr);
	ra.setInputCloud(src_keypoints_ptr);
	ra.setSourceFeatures(src_features_ptr);

	//--- Calculate output
	scalar_cloud registration_output;
	ra.align(registration_output);
	transformation = ra.getFinalTransformation();

	return EXIT_SUCCESS;
}

//#############################################################################
//
//  preciseAlign(): precisely align 2 point cloud
//	
//#############################################################################
int preciseAlign(	const point_cloud		&dst_points,
					const point_cloud 		&src_points,
					Eigen::Matrix4f 		transformation)
{
	//--- Parameters
	const float max_correspondence_distance = 0.05f;
	const float outlier_rejection_threshold = 0.05f;
	const float transformation_epsilon 		= 0;
	const int 	max_iterations 				= 100;

	//--- Setup aligner
	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(max_correspondence_distance);
	icp.setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
	icp.setTransformationEpsilon(transformation_epsilon);
	icp.setMaximumIterations(max_iterations);

	//--- Setup input
	point_cloud::Ptr 	dst_points_ptr(new point_cloud);
	point_cloud::Ptr 	src_points_ptr(new point_cloud);
	*dst_points_ptr = dst_points;
	*src_points_ptr = src_points;
	icp.setInputTarget(dst_points_ptr);
	icp.setInputSource(src_points_ptr);

	//--- Calculate output
	point_cloud registration_output;
	icp.align(registration_output);
	transformation = icp.getFinalTransformation();	

	return EXIT_SUCCESS;
}

} //--- namespace mypcl