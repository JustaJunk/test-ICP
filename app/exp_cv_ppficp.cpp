//#############################################################################
//
//  exp_cv_ppficp.cpp
//
//#############################################################################
#include <iostream>
#include "cv_parser.h"
#include "cv_register.h"

int main()
{
	const std::string 		file_path 		= "../../Dataset/";
	const std::string 		pc_dst_filename = file_path + "input/pc_dst.xyz";
	const std::string 		pc_src_filename = file_path + "input/pc_src.xyz";
	const std::string 		pc_out_filename = file_path + "output/pc_out_cv_ppficp.xyz";
	cv::Mat 				p_dst_mat;
	cv::Mat 				p_src_mat;
	pose_ptr 				out_pose_ptr;
	cv::Mat 				p_out_mat;
	std::clock_t 			time1;
	std::clock_t 			time2;

	//--- Load 2 input point cloud
	std::cout << "Loading " << pc_dst_filename << std::endl;
	if (mycv::loadXYZtoMAT(pc_dst_filename, p_dst_mat) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mycv::loadXYZtoMAT() when loading " << pc_dst_filename << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Loading " << pc_src_filename << std::endl;
	if (mycv::loadXYZtoMAT(pc_src_filename, p_src_mat) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mycv::loadXYZtoMAT() when loading " << pc_src_filename << std::endl;
		return EXIT_FAILURE;
	}

	//--- Apply PPF+ICP (from mycv)
	std::cout << "Applying PPF+ICP algorithm" << std::endl;	
	time1 = std::clock();
	if (mycv::applyPPFICP(	p_dst_mat,
							p_src_mat,
							out_pose_ptr,
							p_out_mat) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mycv::applyICP()" << std::endl;
		return EXIT_FAILURE;
	}
	time2 = std::clock();

	//--- Show result
	std::cout << std::endl  << "runtime: " << (time2-time1)/(double)CLOCKS_PER_SEC 
							<< " sec" << std::endl
							<< "score: " << out_pose_ptr->residual << std::endl 
							<< "transformation:" << std::endl
							<< out_pose_ptr->pose  << std::endl << std::endl;

	//--- Save result
	std::cout << "Saving " << pc_out_filename << std::endl;	
	if (mycv::saveMATtoXYZ(p_out_mat, pc_out_filename) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mycv::saveMATtoXYZ()" << std::endl;		
	}

	return EXIT_SUCCESS;
}

