//#############################################################################
//
//	cv_register.h
//
//#############################################################################
#ifndef __CV_REGISTER_H__
#define __CV_REGISTER_H__

#include <opencv2/core.hpp>

namespace mycv
{

//#############################################################################
//
//  applyICP(): apply ICP algorithm on 2 point clouds
//
//#############################################################################
int applyICP(	const cv::Mat 	&dst_p_mat,
				const cv::Mat 	&src_p_mat,
				double 			score,
				cv::Mat 		&out_p_mat,
				cv::Matx44d 	&transformation);

} //--- namespace mycv

#endif //--- __CV_REGISTER_H__