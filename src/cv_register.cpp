//#############################################################################
//
//	cv_register.cpp
//
//#############################################################################
#include <cstdlib>
#include <iostream>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include "cv_parser.h"
#include "cv_register.h"

namespace mycv
{

//#############################################################################
//
//  applyICP(): apply ICP algorithm on 2 point clouds
//
//#############################################################################
int applyICP(	const cv::Mat 	&dst_p_mat,
				const cv::Mat 	&src_p_mat,
				double 			&score,
				cv::Mat 		&out_p_mat,
				cv::Matx44d 	&transformation)
{
	//--- Compute normals
	cv::Mat 		dst_pn_mat;
	cv::Mat 		src_pn_mat;
	const int 		NbrNum = 10;
	const cv::Vec3f	origin(0.0, 0.0, 0.0);	
	if (cv::ppf_match_3d::computeNormalsPC3d(	dst_p_mat,
												dst_pn_mat,
												NbrNum,
												true,
												origin) != 1)
	{
		std::cout << "ERROR: cv::ppf_match_3d::computeNormalsPC3d() on dst_p_mat" << std::endl;
		return EXIT_FAILURE;
	}
	if (cv::ppf_match_3d::computeNormalsPC3d(	src_p_mat,
												src_pn_mat,
												NbrNum,
												true,
												origin) != 1)
	{
		std::cout << "ERROR: cv::ppf_match_3d::computeNormalsPC3d() on src_p_mat" << std::endl;
		return EXIT_FAILURE;		
	}

	//--- Apply Iterative Closet Point algorithm
	cv::ppf_match_3d::ICP 	icp;
	if (icp.registerModelToScene(	src_pn_mat,
									dst_pn_mat,
									score,
									transformation) != 0)
	{
		std::cout << "ERROR: cv::ppf_match_3d::ICP::registerModelToScene()" << std::endl;
		return EXIT_FAILURE;		
	}
	out_p_mat = cv::ppf_match_3d::transformPCPose(src_p_mat, transformation);

	return EXIT_SUCCESS;
}

//#############################################################################
//
//  applyICP(): apply ICP algorithm on 2 point clouds
//
//#############################################################################
int applyICP(	const std::vector<cv::Point3d> 	&dst_p_vec,
				const std::vector<cv::Point3d> 	&src_p_vec,
				double 							&score,
				cv::Mat 						&out_p_mat,
				cv::Matx44d 					&transformation)
{
	cv::Mat dst_p_mat;
	cv::Mat src_p_mat;

	convertVECtoMAT(dst_p_vec, dst_p_mat);
	convertVECtoMAT(src_p_vec, src_p_mat);

	if (applyICP(	dst_p_mat,
					src_p_mat,
					score,
					out_p_mat,
					transformation) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

} //--- namespace mycv