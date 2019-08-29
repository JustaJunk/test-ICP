//#############################################################################
//
//  exp_cv_icp.cpp
//
//#############################################################################
#include <iostream>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include "cv_parser.h"

int main()
{
	cv::Mat 				p_dst_mat;
	cv::Mat 				p_src_mat;
	cv::Mat 				p_out_mat;
	const std::string 		file_path 		= "../../Dataset/";
	const std::string 		pc_dst_filename = file_path + "input/pc_dst.xyz";
	const std::string 		pc_src_filename = file_path + "input/pc_src.xyz";
	const std::string 		pc_out_filename = file_path + "output/pc_out_cv_icp.xyz";
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

	time1 = std::clock();

	//--- Compute normals
	cv::Mat 		pn_dst_mat;
	cv::Mat 		pn_src_mat;
	const int 		NbrNum = 10;
	const cv::Vec3f	origin(0.0, 0.0, 0.0);
	std::cout << "Computing normals of p_dst_mat" << std::endl;		
	if (cv::ppf_match_3d::computeNormalsPC3d(	p_dst_mat,
												pn_dst_mat,
												NbrNum,
												true,
												origin) != 1)
	{
		std::cout << "ERROR: cv::ppf_match_3d::computeNormalsPC3d() on p_dst_mat" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Computing normals of p_src_mat" << std::endl;	
	if (cv::ppf_match_3d::computeNormalsPC3d(	p_src_mat,
												pn_src_mat,
												NbrNum,
												true,
												origin) != 1)
	{
		std::cout << "ERROR: cv::ppf_match_3d::computeNormalsPC3d() on p_src_mat" << std::endl;
		return EXIT_FAILURE;		
	}

	//--- Apply Iterative Closet Point algorithm
	double 					score;
	cv::Matx44d 			transformation;
	cv::ppf_match_3d::ICP 	icp;
	std::cout << "Applying ICP algorithm" << std::endl;	
	if (icp.registerModelToScene(	pn_src_mat,
									pn_dst_mat,
									score,
									transformation) != 0)
	{
		std::cout << "ERROR: cv::ppf_match_3d::ICP::registerModelToScene()" << std::endl;
		return EXIT_FAILURE;		
	}
	p_out_mat = cv::ppf_match_3d::transformPCPose(p_src_mat, transformation);

	time2 = std::clock();

	//--- Show result
	std::cout << std::endl  << "runtime: " << (time2-time1)/(double)CLOCKS_PER_SEC 
							<< " sec" << std::endl
							<< "score: " << score << std::endl 
							<< "transformation:" << std::endl
							<< transformation  << std::endl << std::endl;

	//--- Save result
	std::cout << "Saving " << pc_out_filename << std::endl;	
	if (mycv::saveMATtoXYZ(p_out_mat, pc_out_filename) == EXIT_FAILURE)
	{
		std::cout << "ERROR: mycv::saveMATtoXYZ()" << std::endl;		
	}

	return EXIT_SUCCESS;
}