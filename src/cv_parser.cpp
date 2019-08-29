//#############################################################################
//
//	cv_parser.cpp
//
//#############################################################################
#include <cstdlib>
#include <iostream>
#include "cv_parser.h"

namespace mycv
{

//#############################################################################
//
//  convertPCVtoMAT(): convert std::vector<cv::Point3d> to cv::Mat (Nx3)
//
//#############################################################################
int convertVECtoMAT(	const std::vector<cv::Point3d> 	&points_vec,
						cv::Mat 						&Nx3_mat)
{
	int point_count = (int)points_vec.size();

	Nx3_mat.release();
	Nx3_mat.create(point_count,3,CV_32F);

	for (int row = 0; row < point_count; ++row)
	{
		Nx3_mat.at<float>(row,0) = (float)points_vec.at(row).x;
		Nx3_mat.at<float>(row,1) = (float)points_vec.at(row).y;
		Nx3_mat.at<float>(row,2) = (float)points_vec.at(row).z;
	}

	return EXIT_SUCCESS;
}


//#############################################################################
//
//  loadXYZtoMAT(): load XYZ file and convert it to cv::Mat (Nx3) object
//
//#############################################################################
int loadXYZtoMAT(	const std::string 	&xyz_filename,
					cv::Mat 			&Nx3_mat)
{	
	std::ifstream 				xyz_fs;
	std::string 				line;
	std::vector<std::string> 	str_vec;
	std::vector<cv::Point3d>	points_vec; 	

	//--- Check that file exists and open successfully
	xyz_fs.open(xyz_filename.c_str(), std::ios::binary);
	if (xyz_fs.fail() || !xyz_fs.is_open())
	{
	  std::cout << "ERROR: Could not find or open file " << xyz_filename.c_str() << std::endl;
	  xyz_fs.close();
	  return EXIT_FAILURE;
	}

	//--- Parse every line (correspond to every point)
	points_vec.clear();
	points_vec.reserve(1000000);
	while (!xyz_fs.eof())
	{
		std::getline(xyz_fs, line);

		//--- Ignore empty lines
		if (line == "")
		{
		  continue;
		}

		//--- Tokenize the line
		boost::trim(line);
		boost::split(str_vec, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
		if (str_vec.size() != 3)
		{
		  continue;
		}
		points_vec.push_back(cv::Point3d( 	std::stod( str_vec.at(0) ),
											std::stod( str_vec.at(1) ),
											std::stod( str_vec.at(2) )));
	}

	convertVECtoMAT(points_vec, Nx3_mat);

	return EXIT_SUCCESS;
}

//#############################################################################
//
//  saveMATtoXYZ(): save cv::Mat (Nx3) object into XYZ format
//
//#############################################################################
int saveMATtoXYZ(	const cv::Mat 		&Nx3_mat,
					const std::string 	&xyz_filename)
{
	std::ofstream 					xyz_file;
	size_t  						point_count = Nx3_mat.rows;

	//--- Check matrix columns
	if (Nx3_mat.cols != 3)
	{
		std::cout << "ERROR: Input matrix must be Nx3" << std::endl;
		return EXIT_FAILURE;
	}

	//--- Open output XYZ file
	xyz_file.open(xyz_filename);
	if (!xyz_file.is_open())
	{
		std::cout << "ERROR: Saving XYZ file failure" << std::endl;
		return EXIT_FAILURE;
	}

	//--- Write in data
	for (size_t row = 0; row < point_count; ++row)
	{
		xyz_file << Nx3_mat.at<float>(row,0) << " "
				 << Nx3_mat.at<float>(row,1) << " " 
				 << Nx3_mat.at<float>(row,2) << "\n";
	}
	xyz_file.close();

	return EXIT_SUCCESS;
}

} //--- namespace mycv
