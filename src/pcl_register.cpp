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
//  featureAlign(): align point cloud by features
//
//#############################################################################
int featureAlign(	const scalar_cloud		&dst_keypoints,
					const feature_cloud 	&dst_features,
					const scalar_cloud 		&src_keypoints,
					const feature_cloud 	&src_features,
					Eigen::Matrix4f 		&transformation)
{
	//--- Parameters
	const float min_sample_distance 		= 0.025f;
	const float max_correspondence_distance = 0.01f;
	const int 	nr_iterations 				= 500;

	//--- Setup aligner
	pcl::SampleConsensusInitialAlignment<	pcl::PointWithScale, 
											pcl::PointWithScale, 
											pcl::FPFHSignature33> 	fa;
	fa.setMinSampleDistance(min_sample_distance);
	fa.setMaxCorrespondenceDistance(max_correspondence_distance);
	fa.setMaximumIterations(nr_iterations);

	//--- Setup input
	scalar_cloud::Ptr  	dst_keypoints_ptr(new scalar_cloud);
	feature_cloud::Ptr 	dst_features_ptr(new feature_cloud);
	scalar_cloud::Ptr 	src_keypoints_ptr(new scalar_cloud);
	feature_cloud::Ptr 	src_features_ptr(new feature_cloud);
	*dst_keypoints_ptr	= dst_keypoints;
	*dst_features_ptr	= dst_features;
	*src_keypoints_ptr	= src_keypoints;
	*src_features_ptr	= src_features;
	fa.setInputTarget(dst_keypoints_ptr);
	fa.setTargetFeatures(dst_features_ptr);
	fa.setInputCloud(src_keypoints_ptr);
	fa.setSourceFeatures(src_features_ptr);

	//--- Calculate output
	scalar_cloud	out_points;
	fa.align(out_points);
	transformation = fa.getFinalTransformation();

	return EXIT_SUCCESS;
}

//#############################################################################
//
//  roughAlign(): roughly align 2 point cloud
//
//#############################################################################
int roughAlign(		const point_cloud		&dst_points,
					const point_cloud 		&src_points,
					point_cloud				&out_points,					
					Eigen::Matrix4f 		&transformation)
{
	std::vector<point_cloud>	pc_vec{dst_points, src_points};
	std::vector<normal_cloud> 	nc_vec;
	std::vector<scalar_cloud> 	kc_vec;
	std::vector<feature_cloud> 	fc_vec;

	estimateNormals(pc_vec, 
					nc_vec);

	estimateSIFT(	nc_vec, 
					kc_vec);

	estimateFPFH(	pc_vec, 
					nc_vec, 
					kc_vec, 
					fc_vec);

	featureAlign(	kc_vec.at(0),
					fc_vec.at(0),
					kc_vec.at(1),
					fc_vec.at(1),
					transformation);

	pcl::transformPointCloud(src_points, out_points, transformation);

	return EXIT_SUCCESS;
}

//#############################################################################
//
//  preciseAlign(): precisely align 2 point cloud
//	
//#############################################################################
int preciseAlign(	const point_cloud		&dst_points,
					const point_cloud 		&src_points,
					point_cloud 			&out_points,
					Eigen::Matrix4f 		&transformation,
					double 					&score)
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
	icp.align(out_points);
	score = icp.getFitnessScore();
	transformation = icp.getFinalTransformation();	

	return EXIT_SUCCESS;
}

//#############################################################################
//
//  hybridAlign(): roughly align and then precisely align
//	
//#############################################################################
int hybridAlign(	const point_cloud		&dst_points,
					const point_cloud 		&src_points,
					point_cloud 			&out_points,
					Eigen::Matrix4f 		&transformation,
					double 					&score)
{
	Eigen::Matrix4f t1;
	Eigen::Matrix4f t2;
	point_cloud 	tmp_points;

	roughAlign(	dst_points, 
				src_points,
				tmp_points, 
				t1);

	preciseAlign(	dst_points,
					tmp_points,
					out_points,
					t2,
					score);

	transformation = t2*t1;

	return EXIT_SUCCESS;
}

} //--- namespace mypcl