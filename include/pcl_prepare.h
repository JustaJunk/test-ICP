//#############################################################################
//
//	pcl_prepare.h
//
//#############################################################################
#ifndef __PCL_PREPARE_H__
#define __PCL_PREPARE_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

using point_cloud 	= pcl::PointCloud<pcl::PointXYZ>;
using normal_cloud 	= pcl::PointCloud<pcl::PointNormal>;
using scalar_cloud 	= pcl::PointCloud<pcl::PointWithScale>;
using feature_cloud	= pcl::PointCloud<pcl::FPFHSignature33>;

namespace mypcl
{

//#############################################################################
//
//  estimateNormals(): estimate normals of the input point cloud
//
//#############################################################################
int estimateNormals(const std::vector<point_cloud> 		&points_vec,
					std::vector<normal_cloud> 			&normals_vec);

//#############################################################################
//
//  estimateSIFT(): estimate SIFT keypoints of the input point cloud
//
//#############################################################################
int estimateSIFT(	const std::vector<normal_cloud> 	&normals_vec,
					std::vector<scalar_cloud> 			&keypoints_vec);

//#############################################################################
//
//  estimateFPFH(): estimate FPFH features from SIFT keypoints
//
//#############################################################################
int estimateFPFH(	const std::vector<point_cloud>		&points_vec,
					const std::vector<normal_cloud> 	&normals_vec,
					const std::vector<scalar_cloud> 	&keypoints_vec,
					std::vector<feature_cloud> 			&features_vec);

} //--- namespace mypcl

#endif //--- __PCL_PREPARE_H__