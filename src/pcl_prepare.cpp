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
//  samplePointCloud(): down sample the input point cloud
//
//#############################################################################
int samplePointCloud(	const std::vector<point_cloud> 		&points_vec,
						std::vector<point_cloud> 			&samples_vec)
{
	//--- Check if input vector is empty
	if (points_vec.empty())
	{
		std::cout << "ERROR: Input point-cloud vector is empty" << std::endl;
		return EXIT_FAILURE;
	}

	//--- Parameters
	const float 	leaf_size 	= 0.01f;
	const size_t 	cloud_count = points_vec.size();

	//--- Setup sampler
	pcl::VoxelGrid<pcl::PointXYZ> sp;
	sp.setLeafSize(leaf_size,leaf_size,leaf_size);

	//--- Setup variables for iterations
	point_cloud::Ptr 	points_ptr(new point_cloud);
	point_cloud 		samples;

	//--- Sample the point cloud
	samples_vec.clear();
	samples_vec.resize(cloud_count);
	for (size_t cloud_index = 0; cloud_index < cloud_count; ++cloud_index)
	{
		//--- Input
		*points_ptr = points_vec.at(cloud_index);
		sp.setInputCloud(points_ptr);

		//--- Output
		sp.filter(samples);
		samples_vec.at(cloud_index) = samples;
	}	
	return EXIT_SUCCESS;
}

//#############################################################################
//
//  estimateNormals(): estimate normals of the input point cloud
//
//#############################################################################
int estimateNormals(const std::vector<point_cloud> 		&points_vec,
					std::vector<normal_cloud> 			&normals_vec)
{
	//--- Check if input vector is empty
	if (points_vec.empty())
	{
		std::cout << "ERROR: Input point-cloud vector is empty" << std::endl;
		return EXIT_FAILURE;
	}

	//--- Parameters
	const double 	radius 		= 0.05;
	const size_t 	cloud_count = points_vec.size();

	//--- Setup estimator
	pcl::NormalEstimation<pcl::PointXYZ,pcl::PointNormal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree_xyz);
	ne.setRadiusSearch(radius);

	//--- Setup variables for iterations
	point_cloud::Ptr 	points_ptr(new point_cloud);
	normal_cloud 		normals;
	std::vector<int> 	nan_idx;
	size_t 				normal_count;

	//--- Estimate normals for every point cloud
	normals_vec.clear();
	normals_vec.resize(cloud_count);
	for (size_t cloud_index = 0; cloud_index < cloud_count; ++cloud_index)
	{
		//--- Input
		*points_ptr = points_vec.at(cloud_index);
		pcl::removeNaNFromPointCloud(*points_ptr, *points_ptr, nan_idx);
		ne.setInputCloud(points_ptr);

		//---Output
		ne.compute(normals);
		normal_count = normals.points.size();
		for (size_t i = 0;  i < normal_count; ++i) 
		{
			normals.points.at(i).x = points_ptr->points.at(i).x;
			normals.points.at(i).y = points_ptr->points.at(i).y;
			normals.points.at(i).z = points_ptr->points.at(i).z;
		}
		normals_vec.at(cloud_index) = normals;
	}
	return EXIT_SUCCESS;
}

//#############################################################################
//
//  estimateSIFT(): estimate SIFT keypoints of the input normal cloud
//
//#############################################################################
int estimateSIFT(	const std::vector<normal_cloud> 	&normals_vec,
					std::vector<scalar_cloud> 			&keypoints_vec)
{
	//--- Check if input vector is empty
	if (normals_vec.empty())
	{
		std::cout << "ERROR: Input normal-cloud vector is empty" << std::endl;
		return EXIT_FAILURE;
	}

	//--- Parameters
	const float 	min_scale 			= 0.01f;
	const int 		n_octaves 			= 3;
	const int 		n_scales_per_octave = 4;
	const float 	min_contrast 		= 0.001f;
	const size_t 	cloud_count 		= normals_vec.size();

	//--- Setup estimator
	pcl::SIFTKeypoint<pcl::PointNormal,pcl::PointWithScale> sift;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointNormal>());
	sift.setSearchMethod(tree_normal);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);

	//--- Setup variables for iterations
	normal_cloud::Ptr 	normals_ptr(new normal_cloud);	
	scalar_cloud 		keypoints;

	//--- Estimate keypoints for every normal cloud
	keypoints_vec.clear();
	keypoints_vec.resize(cloud_count);
	for (size_t cloud_index = 0; cloud_index < cloud_count; ++cloud_index)
	{
		//--- Input
		*normals_ptr = normals_vec.at(cloud_index);
		sift.setInputCloud(normals_ptr);

		//--- Output
		sift.compute(keypoints);
		keypoints_vec.at(cloud_index) = keypoints;
	}
	return EXIT_SUCCESS;
}

//#############################################################################
//
//  estimateFPFH(): estimate FPFH features from SIFT keypoints
//
//#############################################################################
int estimateFPFH(	const std::vector<point_cloud>		&points_vec,
					const std::vector<normal_cloud> 	&normals_vec,
					const std::vector<scalar_cloud> 	&keypoints_vec,
					std::vector<feature_cloud> 			&features_vec)
{
	//--- Check if input vectors are empty
	if (points_vec.empty())
	{
		std::cout << "ERROR: Input point-cloud vector is empty" << std::endl;
		return EXIT_FAILURE;
	}
	if (normals_vec.empty())
	{
		std::cout << "ERROR: Input normal-cloud vector is empty" << std::endl;
		return EXIT_FAILURE;
	}
	if (keypoints_vec.empty())
	{
		std::cout << "ERROR: Input keypoint-cloud vector is empty" << std::endl;
		return EXIT_FAILURE;
	}

	//--- Check if 3 input vectors are same size
	if (points_vec.size()  != normals_vec.size() ||
		normals_vec.size() != keypoints_vec.size())
	{
		std::cout << "ERROR: Input cloud count not match" << std::endl;
		return EXIT_FAILURE;
	}

	//--- Parameters
	const double radius 		= 0.05;
	const size_t cloud_count 	= points_vec.size();

	//--- Setup estimator
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz(new pcl::search::KdTree<pcl::PointXYZ>());
	fpfh.setSearchMethod(tree_xyz);
	fpfh.setRadiusSearch(radius);

	//--- Setup variables for iterations
	point_cloud::Ptr 	points_ptr(new point_cloud);
	normal_cloud::Ptr 	normals_ptr(new normal_cloud);
	point_cloud::Ptr 	keypoints_ptr(new point_cloud);
	feature_cloud 		features;

	//--- Estimate FPFH
	features_vec.clear();
	features_vec.resize(cloud_count);
	for (size_t cloud_index = 0; cloud_index < cloud_count; ++cloud_index)
	{
		//--- Input
		*points_ptr  	= points_vec.at(cloud_index);
		*normals_ptr 	= normals_vec.at(cloud_index);
		pcl::copyPointCloud(keypoints_vec.at(cloud_index), *keypoints_ptr);
		fpfh.setSearchSurface(points_ptr);
		fpfh.setInputNormals(normals_ptr);
		fpfh.setInputCloud(keypoints_ptr);

		//--- Output
		fpfh.compute(features);
		features_vec.at(cloud_index) = features;
	}
	return EXIT_SUCCESS;
}

} //--- namespace mypcl