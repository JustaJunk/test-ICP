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
#  icp_exp_pcl 
#------------------------------------------------------------------------------
add_executable(
	icp_exp_pcl 
	icp_exp_pcl.cpp
)
target_link_libraries(
	icp_exp_pcl 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  icp_exp_cv 
#------------------------------------------------------------------------------
add_executable(
	icp_exp_cv 
	icp_exp_cv.cpp
)
target_link_libraries(
	icp_exp_cv 
	${LIBRARIES}
)