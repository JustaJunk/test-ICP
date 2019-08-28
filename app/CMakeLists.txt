#------------------------------------------------------------------------------
#  gen_random_pcd 
#------------------------------------------------------------------------------
add_executable(	
	gen_random_pcd 			
	gen_random_pcd.cpp
)
target_link_libraries(
	gen_random_pcd 	
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  file_pcd2xyz 
#------------------------------------------------------------------------------
add_executable(
	file_pcd2xyz 
	file_pcd2xyz.cpp
)
target_link_libraries(
	file_pcd2xyz 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  file_xyz2pcd 
#------------------------------------------------------------------------------
add_executable(
	file_xyz2pcd 
	file_xyz2pcd.cpp
)
target_link_libraries(
	file_xyz2pcd 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  pcl_icp_random 
#------------------------------------------------------------------------------
add_executable(
	pcl_icp_random 
	pcl_icp_random.cpp
)
target_link_libraries(
	pcl_icp_random 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  pcl_icp_exp 
#------------------------------------------------------------------------------
add_executable(
	pcl_icp_exp 
	pcl_icp_exp.cpp
)
target_link_libraries(
	pcl_icp_exp 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  cv_icp_exp 
#------------------------------------------------------------------------------
add_executable(
	cv_icp_exp 
	cv_icp_exp.cpp
)
target_link_libraries(
	cv_icp_exp 
	${LIBRARIES}
)