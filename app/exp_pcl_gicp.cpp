//#############################################################################
//
//  exp_pcl_gicp.cpp
//
//#############################################################################
#include <iostream>
#include <pcl/registration/gicp.h>
#include "pcl_parser.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pcd_dst;
	pcl::PointCloud<pcl::PointXYZ> 	pcd_src;
	pcl::PointCloud<pcl::PointXYZ> 	pcd_out;
	const std::string 				file_path 		= "../../Dataset/";
	const std::string 				pc_dst_filename = file_path + "input/pc_dst.xyz";
	const std::string 				pc_src_filename = file_path + "input/pc_src.xyz";
	const std::string 				pc_out_filename = file_path + "output/pc_out_pcl_gicp.xyz";

	//--- Load 2 input point cloud
	std::cout << "Loading " << pc_dst_filename << std::endl;
	if (mypcl::loadXYZtoPCD(pc_dst_filename, pcd_dst) == EXIT_FAILURE)
	{
		std::cout << "ERROR: loadXYZtoPCD() when loading" << pc_dst_filename << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Loading " << pc_src_filename << std::endl;
	if (mypcl::loadXYZtoPCD(pc_src_filename, pcd_src) == EXIT_FAILURE)
	{
		std::cout << "ERROR: loadXYZtoPCD() when loading" << pc_src_filename << std::endl;
		return EXIT_FAILURE;
	}

	//--- Apply Generalized Iterative Closet Point algorithm
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_dst_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	*pcd_dst_ptr = pcd_dst;
	*pcd_src_ptr = pcd_src;
	gicp.setInputTarget(pcd_dst_ptr);
	gicp.setInputSource(pcd_src_ptr);
	std::cout << "Applying GICP algorithm" << std::endl;
	gicp.align(pcd_out);

	//--- Show result
	std::cout << std::endl  << "score: " << gicp.getFitnessScore() << std::endl
							<< "transformation: " << std::endl
							<< gicp.getFinalTransformation() << std::endl << std::endl;

	//--- Save point cloud
	std::cout << "Saving " << pc_out_filename << std::endl;	
	if (mypcl::savePCDtoXYZ(pcd_out, pc_out_filename) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mypcl::savePCDtoXYZ()" << std::endl;
		return EXIT_FAILURE;			
	}

	return EXIT_SUCCESS;
}