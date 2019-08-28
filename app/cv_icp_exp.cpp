//#############################################################################
//
//  cv_icp_exp.cpp
//
//#############################################################################
#include <iostream>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include "pcd_xyz.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZ> 	pc1;
	pcl::PointCloud<pcl::PointXYZ> 	pc2;
	pcl::PointCloud<pcl::PointXYZ> 	pc2r;
	std::string 					pc1_filename  = "pc_1.xyz";
	std::string 					pc2_filename  = "pc_2.xyz";
	std::string 					pc2r_filename = "pc_2r.xyz";

	//--- Load 2 input point cloud
	if (mypcl::loadXYZtoPCD(pc1_filename, pc1) == EXIT_SUCCESS)
	{
		std::cout << "COMPLETE: load " << pc1_filename << std::endl;
	}
	else
	{
		std::cout << "ERROR: loadXYZtoPCD() when loading " << pc1_filename << std::endl;
		return EXIT_FAILURE;
	}
	if (mypcl::loadXYZtoPCD(pc2_filename, pc2) == EXIT_SUCCESS)
	{
		std::cout << "COMPLETE: load " << pc2_filename << std::endl;
	}
	else
	{
		std::cout << "ERROR: loadXYZtoPCD() when loading " << pc2_filename << std::endl;
		return EXIT_FAILURE;
	}

	//--- Convert pc into cv::Mat
	size_t 		pc1_count 	= pc1.size();
	size_t 		pc2_count 	= pc2.size();
	cv::Mat 	p1_mat(pc1_count, 3, CV_32F);
	cv::Mat 	p2_mat(pc2_count, 3, CV_32F);
	for (size_t row = 0; row < pc1_count; ++row)
	{
		p1_mat.at<float>(row,0) = pc1.points.at(row).x;
		p1_mat.at<float>(row,1) = pc1.points.at(row).y;
		p1_mat.at<float>(row,2) = pc1.points.at(row).z;
	}
	std::cout << "COMPLETE: convert pc1 to p1_mat " << std::endl;
	for (size_t row = 0; row < pc2_count; ++row)
	{
		p2_mat.at<float>(row,0) = pc2.points.at(row).x;
		p2_mat.at<float>(row,1) = pc2.points.at(row).y;
		p2_mat.at<float>(row,2) = pc2.points.at(row).z;
	}
	std::cout << "COMPLETE: convert pc2 to p2_mat " << std::endl;

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

	return EXIT_SUCCESS;
}