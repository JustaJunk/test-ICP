//#############################################################################
//
//  file_xyz2pcd.cpp
//
//#############################################################################
#include <cstdlib>
#include <iostream>
#include "pcl_parser.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pc;
	std::string  					filename 	 = "random_pc";
	std::string  					pcd_filename = filename + "_copy.pcd";
	std::string  					xyz_filename = filename + ".xyz";

	//--- Load XYZ file to pc
	if (mypcl::loadXYZtoPCD(xyz_filename, pc) == EXIT_FAILURE)
	{
		std::cout << "ERROR: loadXYZtoPCD()" << std::endl;
		return EXIT_FAILURE;
	}

	//--- Save pc in PCD file
	if (pcl::io::savePCDFile(pcd_filename, pc) == -1)
	{
		std::cout << "ERROR: Can't save PCD file " << pcd_filename << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}