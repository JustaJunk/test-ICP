//#############################################################################
//
//  file_pcd2xyz.cpp
//
//#############################################################################
#include <cstdlib>
#include <iostream>
#include "pcd_xyz.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pc;
	std::string  					filename 	 = "random_pc";
	std::string  					pcd_filename = filename + ".pcd";
	std::string  					xyz_filename = filename + ".xyz";

	//--- Load PCD file to pc
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename, pc) == -1)
	{
		std::cout << "ERROR: Can't find PCD file " << pcd_filename << std::endl;
		return EXIT_FAILURE;
	}

	//--- Save pc in XYZ file
	if (mypcl::savePCDtoXYZ(pc, xyz_filename) == EXIT_FAILURE)
	{
		std::cout << "ERROR: savePCDtoXYZ()" << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}