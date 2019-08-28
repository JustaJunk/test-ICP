//#############################################################################
//
//  pcl_icp_exp.cpp
//
//#############################################################################
#include <iostream>
#include <pcl/registration/icp.h>
#include "pcd_xyz.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pc1;
	pcl::PointCloud<pcl::PointXYZ> 	pc2;
	pcl::PointCloud<pcl::PointXYZ> 	pc2r;
	std::string 					pc1_filename  = "pc_1.xyz";
	std::string 					pc2_filename  = "pc_2.xyz";
	std::string 					pc2r_filename = "pc_2r.xyz";

	//--- Load 2 input point cloud
	if (mypcl::loadXYZtoPCD(pc1_filename, pc1) == EXIT_FAILURE)
	{
		std::cout << "ERROR: loadXYZtoPCD() when loading" << pc1_filename << std::endl;
		return EXIT_FAILURE;
	}
	if (mypcl::loadXYZtoPCD(pc2_filename, pc2) == EXIT_FAILURE)
	{
		std::cout << "ERROR: loadXYZtoPCD() when loading" << pc2_filename << std::endl;
		return EXIT_FAILURE;
	}

	//--- Apply iterative closet point algorithm
	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	*pc1_ptr = pc1;
	*pc2_ptr = pc2;
	icp.setInputTarget(pc1_ptr);
	icp.setInputSource(pc2_ptr);
	icp.align(pc2r);

	//--- Show result
	std::cout 	<< "score: " << icp.getFitnessScore() << std::endl
				<< "transformation: " << std::endl
				<< icp.getFinalTransformation() << std::endl;

	//--- Save point cloud
	mypcl::savePCDtoXYZ(pc2r, pc2r_filename);

	return EXIT_SUCCESS;
}