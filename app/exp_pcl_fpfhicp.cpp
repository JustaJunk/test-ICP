//#############################################################################
//
//  exp_pcl_fpfhicp.cpp
//
//#############################################################################
#include <iostream>
#include <pcl/registration/icp.h>
#include "pcl_parser.h"
#include "pcl_register.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pcd_dst;
	pcl::PointCloud<pcl::PointXYZ> 	pcd_src;
	pcl::PointCloud<pcl::PointXYZ> 	pcd_out;
	const std::string 				file_path 		= "../../Dataset/";
	const std::string 				pc_dst_filename = file_path + "input/pc_dst.xyz";
	const std::string 				pc_src_filename = file_path + "input/pc_src.xyz";
	const std::string 				pc_out_filename = file_path + "output/pc_out_pcl_fpfhicp.xyz";
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

	//--- Sample the point cloud
	std::vector<point_cloud> 	pc{pcd_dst, pcd_src};
	std::vector<point_cloud>	spc;
	mypcl::samplePointCloud(pc, spc);

	//--- Apply FPFH + ICP
	Eigen::Matrix4f transformation;
	double 			score;
	std::cout << "Applying ICP algorithm" << std::endl;
	time1 = std::clock();
	mypcl::hybridAlign(	spc.at(0),
						spc.at(1),
						pcd_out,
						transformation,
						score);
	pcl::transformPointCloud(pcd_src, pcd_out, transformation);	
	time2 = std::clock();
	//--- Show result
	std::cout << std::endl  << "runtime: " << (time2-time1)/(double)CLOCKS_PER_SEC
							<< " sec" << std::endl
							<< "score: " << score << std::endl
							<< "transformation:" << std::endl
							<< transformation << std::endl << std::endl;

	//--- Save point cloud
	std::cout << "Saving " << pc_out_filename << std::endl;	
	if (mypcl::savePCDtoXYZ(pcd_out, pc_out_filename) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mypcl::savePCDtoXYZ()" << std::endl;
		return EXIT_FAILURE;			
	}

	return EXIT_SUCCESS;
}