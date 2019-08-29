//#############################################################################
//
//	pcl_parser.h
//
//#############################################################################
#ifndef __PCL_PARSER_H__
#define __PCL_PARSER_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace mypcl
{

//#############################################################################
//
//  loadXYZtoPCD(): load XYZ file and convert it to pcl::PointCloud object
//
//#############################################################################
int loadXYZtoPCD(	const std::string 						&xyz_filename,
					pcl::PointCloud<pcl::PointXYZ> 			&pcl_pc);

//#############################################################################
//
//  savePCDtoXYZ(): save pcl::PointCloud object into XYZ format
//
//#############################################################################
int savePCDtoXYZ(	const pcl::PointCloud<pcl::PointXYZ> 	&pcl_pc,
					const std::string 						&xyz_filename);

} //--- namespace mypcl

#endif //--- __PCL_PARSER_H__
