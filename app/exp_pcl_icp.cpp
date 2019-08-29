//#############################################################################
//
//  exp_pcl_icp.cpp
//
//#############################################################################
#include <iostream>
#include <pcl/registration/icp.h>
#include "pcl_parser.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pcd_dst;
	pcl::PointCloud<pcl::PointXYZ> 	pcd_src;
	pcl::PointCloud<pcl::PointXYZ> 	pcd_out;
	const std::string 				file_path 		= "../../Dataset/";
	const std::string 				pc_dst_filename = file_path + "input/pc_dst.xyz";
	const std::string 				pc_src_filename = file_path + "input/pc_src.xyz";
	const std::string 				pc_out_filename = file_path + "output/pc_out_pcl_icp.xyz";
	std::clock_t 					time1;
	std::clock_t 					time2;

	//--- Load 2 input point cloud
	std::cout << "Loading " << pc_dst_filename << std::endl;
	if (mypcl::loadXYZtoPCD(pc_dst_filename, pcd_dst) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mypcl::loadXYZtoPCD() when loading" << pc_dst_filename << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Loading " << pc_src_filename << std::endl;
	if (mypcl::loadXYZtoPCD(pc_src_filename, pcd_src) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mypcl::loadXYZtoPCD() when loading" << pc_src_filename << std::endl;
		return EXIT_FAILURE;
	}

	//--- Apply Iterative Closet Point algorithm
	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_dst_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	*pcd_dst_ptr = pcd_dst;
	*pcd_src_ptr = pcd_src;
	icp.setInputTarget(pcd_dst_ptr);
	icp.setInputSource(pcd_src_ptr);
	std::cout << "Applying ICP algorithm" << std::endl;
	time1 = std::clock();
	icp.align(pcd_out);
	time2 = std::clock();
	//--- Show result
	std::cout << std::endl  << "runtime: " << (time2-time1)/(double)CLOCKS_PER_SEC
							<< " sec" << std::endl
							<< "score: " << icp.getFitnessScore() << std::endl
							<< "transformation: " << std::endl
							<< icp.getFinalTransformation() << std::endl << std::endl;

	//--- Save point cloud
	std::cout << "Saving " << pc_out_filename << std::endl;	
	if (mypcl::savePCDtoXYZ(pcd_out, pc_out_filename) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mypcl::savePCDtoXYZ()" << std::endl;
		return EXIT_FAILURE;			
	}

	return EXIT_SUCCESS;
}