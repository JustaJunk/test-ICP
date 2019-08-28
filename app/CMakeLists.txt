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
#  pcl_icp 
#------------------------------------------------------------------------------
add_executable(
	pcl_icp 
	pcl_icp.cpp
)
target_link_libraries(
	pcl_icp 
	${LIBRARIES}
)