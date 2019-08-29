//#############################################################################
//
//  gen_random_pcd.cpp
//
//#############################################################################
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pc;
	size_t 							pc_size;

	//--- Setup point cloud pc
	pc.width	= 100000;
	pc.height	= 1;
	pc.is_dense	= true;
	pc_size 	= pc.width * pc.height;
	pc.points.resize(pc_size);

	//--- Fill in pc with points of random coordinates
	for (size_t i = 0; i < pc_size; ++i)
	{
		pc.points.at(i).x = 1024*std::rand()/(RAND_MAX+1.0f);
		pc.points.at(i).y = 1024*std::rand()/(RAND_MAX+1.0f);
		pc.points.at(i).z = 1024*std::rand()/(RAND_MAX+1.0f);
	}

	//--- Save PCD file
	pcl::io::savePCDFileASCII("random_pc.pcd", pc);

	return EXIT_SUCCESS;
}