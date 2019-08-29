//#############################################################################
//
//  icp_exp_pcl.cpp
//
//#############################################################################
#include <iostream>
#include <pcl/registration/icp.h>
#include "pcl_parser.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pcd1;
	pcl::PointCloud<pcl::PointXYZ> 	pcd2;
	pcl::PointCloud<pcl::PointXYZ> 	pcd2r;
	const std::string 				file_path 		= "../../Dataset/";
	const std::string 				pc1_filename  	= file_path + "pc_1.xyz";
	const std::string 				pc2_filename  	= file_path + "pc_2.xyz";
	const std::string 				pc2r_filename 	= file_path + "pc_2_pcl.xyz";

	//--- Load 2 input point cloud
	if (mypcl::loadXYZtoPCD(pc1_filename, pcd1) == EXIT_FAILURE)
	{
		std::cout << "ERROR: loadXYZtoPCD() when loading" << pc1_filename << std::endl;
		return EXIT_FAILURE;
	}
	if (mypcl::loadXYZtoPCD(pc2_filename, pcd2) == EXIT_FAILURE)
	{
		std::cout << "ERROR: loadXYZtoPCD() when loading" << pc2_filename << std::endl;
		return EXIT_FAILURE;
	}

	//--- Apply iterative closet point algorithm
	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd2_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	*pcd1_ptr = pcd1;
	*pcd2_ptr = pcd2;
	icp.setInputTarget(pcd1_ptr);
	icp.setInputSource(pcd2_ptr);
	icp.align(pcd2r);

	//--- Show result
	std::cout 	<< "score: " << icp.getFitnessScore() << std::endl
				<< "transformation: " << std::endl
				<< icp.getFinalTransformation() << std::endl;

	//--- Save point cloud
	mypcl::savePCDtoXYZ(pcd2r, pc2r_filename);

	return EXIT_SUCCESS;
}