//#############################################################################
//
//  pcl_icp_random.cpp
//
//#############################################################################
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr 	pc_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr 	pc_dst_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> 			pc_rlt;
	size_t  								pc_size;

	//--- Setup point cloud pc_src
	pc_src_ptr->width 		= 100;
	pc_src_ptr->height 		= 1;
	pc_src_ptr->is_dense 	= true;
	pc_size  				= pc_src_ptr->width * pc_src_ptr->height;
	pc_src_ptr->points.resize(pc_size);

	//--- Fill in pc_src with points of random coordinates
	for (size_t i = 0; i < pc_size; ++i)
	{
		pc_src_ptr->points.at(i).x = 1024*std::rand()/(RAND_MAX+1.0f);
		pc_src_ptr->points.at(i).y = 1024*std::rand()/(RAND_MAX+1.0f);
		pc_src_ptr->points.at(i).z = 1024*std::rand()/(RAND_MAX+1.0f);
	}

	//--- Shift pc_src to get pc_dst
	*pc_dst_ptr = *pc_src_ptr;
	for (size_t i = 0; i < pc_size; ++i)
	{
		pc_dst_ptr->points.at(i).x = pc_dst_ptr->points.at(i).x + 0.7f;
		pc_dst_ptr->points.at(i).y = pc_dst_ptr->points.at(i).y + 0.3f;
		pc_dst_ptr->points.at(i).z = pc_dst_ptr->points.at(i).z + 0.5f;		
	}

	//--- Apply iterative closet point algorithm
	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
	icp.setInputSource(pc_src_ptr);
	icp.setInputTarget(pc_dst_ptr);
	icp.align(pc_rlt);

	//--- Show result
	std::cout 	<< "score: " << icp.getFitnessScore() << std::endl
				<< "transformation: " << std::endl
				<< icp.getFinalTransformation() << std::endl;

	return EXIT_SUCCESS;
}