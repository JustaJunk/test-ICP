//#############################################################################
//
//	cv_register.h
//
//#############################################################################
#ifndef __CV_REGISTER_H__
#define __CV_REGISTER_H__

#include <opencv2/core.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>

using pose_ptr = cv::ppf_match_3d::Pose3DPtr;

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
				cv::Matx44d 	&transformation);

//#############################################################################
//
//  applyICP(): apply ICP algorithm on 2 point clouds
//
//#############################################################################
int applyICP(	const std::vector<cv::Point3d> 	&dst_p_vec,
				const std::vector<cv::Point3d> 	&src_p_vec,
				double 							&score,
				cv::Mat 						&out_p_mat,
				cv::Matx44d 					&transformation);

//#############################################################################
//
//  applyPPF(): apply PPF to train model
//
//#############################################################################
int applyPPF(	const cv::Mat 			&dst_p_mat,
				const cv::Mat 			&src_p_mat,
				std::vector<pose_ptr> 	&pose_ptrs);

//#############################################################################
//
//  applyPPFICP(): apply PPF+ICP algorithm on 2 point clouds
//
//#############################################################################
int applyPPFICP(const cv::Mat 			&dst_p_mat,
				const cv::Mat 			&src_p_mat,
				const int 				&head,
				std::vector<pose_ptr> 	&pose_ptrs,
				std::vector<cv::Mat>	&out_p_mats);

//#############################################################################
//
//  applyPPFICP(): apply PPF+ICP algorithm on 2 point clouds
//
//#############################################################################
int applyPPFICP(const cv::Mat 			&dst_p_mat,
				const cv::Mat 			&src_p_mat,
				pose_ptr 				&first_pose_ptr,
				cv::Mat					&out_p_mat);

} //--- namespace mycv

#endif //--- __CV_REGISTER_H__