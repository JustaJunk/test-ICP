//#############################################################################
//
//	pcd_xyz.h
//
//#############################################################################
#ifndef __PCD_XYZ_H__
#define __PCD_XYZ_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace mypcl
{

//#############################################################################
//
//  loadXYZtoPCD(): read XYZ file and convert it to PCD object
//
//#############################################################################
int loadXYZtoPCD(	const std::string 						&xyz_filename,
					pcl::PointCloud<pcl::PointXYZ> 			&pcl_pc);

//#############################################################################
//
//  savePCDtoXYZ(): save PCD object into XYZ format
//
//#############################################################################
int savePCDtoXYZ(	const pcl::PointCloud<pcl::PointXYZ> 	&pcl_pc,
					const std::string 						&xyz_filename);

} //--- namespace mypcl

#endif //--- __PCD_XYZ_H__
