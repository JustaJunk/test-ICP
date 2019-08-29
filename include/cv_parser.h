//#############################################################################
//
//	cv_parser.h
//
//#############################################################################
#ifndef __CV_PARSER_H__
#define __CV_PARSER_H__

#include <opencv2/core.hpp>
#include <pcl/io/pcd_io.h>

namespace mycv
{

//#############################################################################
//
//  convertVECtoMAT(): convert std::vector<cv::Point3d> to cv::Mat (Nx3)
//
//#############################################################################
int convertVECtoMAT(	const std::vector<cv::Point3d> 	&points_vec,
						cv::Mat 						&Nx3_mat);

//#############################################################################
//
//  loadXYZtoMAT(): load XYZ file and convert it to cv::Mat (Nx3) object
//
//#############################################################################
int loadXYZtoMAT(	const std::string 	&xyz_filename,
					cv::Mat 			&Nx3_mat);

//#############################################################################
//
//  saveMATtoXYZ(): save cv::Mat (Nx3) object into XYZ format
//
//#############################################################################
int saveMATtoXYZ(	const cv::Mat 		&Nx3_mat,
					const std::string 	&xyz_filename);

} //--- namespace mycv

#endif //--- __CV_PARSER_H__