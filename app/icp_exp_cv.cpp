//#############################################################################
//
//  icp_exp_cv.cpp
//
//#############################################################################
#include <iostream>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include "cv_parser.h"

int main()
{
	cv::Mat 				p1_mat;
	cv::Mat 				p2_mat;
	cv::Mat 				p2r_mat;
	const std::string 		file_path 		= "../../Dataset/";
	const std::string 		pc1_filename  	= file_path + "pc_1.xyz";
	const std::string 		pc2_filename  	= file_path + "pc_2.xyz";
	const std::string 		pc2r_filename 	= file_path + "pc_2_cv.xyz";

	//--- Load 2 input point cloud
	if (mycv::loadXYZtoMAT(pc1_filename, p1_mat) == EXIT_SUCCESS)
	{
		std::cout << "COMPLETE: load " << pc1_filename << std::endl;
	}
	else
	{
		std::cout << "ERROR: mycv::loadXYZtoMAT() when loading " << pc1_filename << std::endl;
		return EXIT_FAILURE;
	}
	if (mycv::loadXYZtoMAT(pc2_filename, p2_mat) == EXIT_SUCCESS)
	{
		std::cout << "COMPLETE: load " << pc2_filename << std::endl;
	}
	else
	{
		std::cout << "ERROR: mycv::loadXYZtoMAT() when loading " << pc2_filename << std::endl;
		return EXIT_FAILURE;
	}

	//--- Compute normals
	cv::Mat 		pn1_mat;
	cv::Mat 		pn2_mat;
	const int 		NbrNum = 10;
	const cv::Vec3f	origin(0.0, 0.0, 0.0);
	if (cv::ppf_match_3d::computeNormalsPC3d(	p1_mat,
												pn1_mat,
												NbrNum,
												true,
												origin) == 1)
	{
		std::cout << "COMPLETE: compute normals of p1_mat" << std::endl;		
	}
	else
	{
		std::cout << "ERROR: cv::ppf_match_3d::computeNormalsPC3d() on p1_mat" << std::endl;
		return EXIT_FAILURE;
	}
	if (cv::ppf_match_3d::computeNormalsPC3d(	p2_mat,
												pn2_mat,
												NbrNum,
												true,
												origin) == 1)
	{
		std::cout << "COMPLETE: compute normals of p2_mat" << std::endl;		
	}
	else
	{
		std::cout << "ERROR: cv::ppf_match_3d::computeNormalsPC3d() on p2_mat" << std::endl;
		return EXIT_FAILURE;		
	}

	//--- Apply ICP
	double 					score;
	cv::Matx44d 			transformation;
	cv::ppf_match_3d::ICP 	icp;
	if (icp.registerModelToScene(	pn2_mat,
									pn1_mat,
									score,
									transformation) == 0)
	{
		std::cout << "COMPLETE: ICP algorithm" << std::endl;		
	}
	else
	{
		std::cout << "ERROR: cv::ppf_match_3d::ICP::registerModelToScene()" << std::endl;
		return EXIT_FAILURE;		
	}

	//--- Show result
	std::cout << std::endl << "score: " << score << std::endl 
							<< "transformation:" << std::endl
							<< transformation << std::endl << std::endl;

	//--- Save result
	p2r_mat = cv::ppf_match_3d::transformPCPose(p2_mat,transformation);
	mycv::saveMATtoXYZ(p2r_mat, pc2r_filename);

	return EXIT_SUCCESS;
}