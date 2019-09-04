#------------------------------------------------------------------------------
#  test_gen_pcd 
#------------------------------------------------------------------------------
add_executable(	
	test_gen_pcd 			
	test_gen_pcd.cpp
)
target_link_libraries(
	test_gen_pcd 	
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  test_pcd2xyz 
#------------------------------------------------------------------------------
add_executable(
	test_pcd2xyz 
	test_pcd2xyz.cpp
)
target_link_libraries(
	test_pcd2xyz 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  test_xyz2pcd 
#------------------------------------------------------------------------------
add_executable(
	test_xyz2pcd 
	test_xyz2pcd.cpp
)
target_link_libraries(
	test_xyz2pcd 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  exp_cv_icp 
#------------------------------------------------------------------------------
add_executable(
	exp_cv_icp 
	exp_cv_icp.cpp
)
target_link_libraries(
	exp_cv_icp 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  exp_cv_ppficp 
#------------------------------------------------------------------------------
add_executable(
	exp_cv_ppficp 
	exp_cv_ppficp.cpp
)
target_link_libraries(
	exp_cv_ppficp 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  exp_pcl_icp 
#------------------------------------------------------------------------------
add_executable(
	exp_pcl_icp 
	exp_pcl_icp.cpp
)
target_link_libraries(
	exp_pcl_icp 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  exp_pcl_gicp 
#------------------------------------------------------------------------------
add_executable(
	exp_pcl_gicp 
	exp_pcl_gicp.cpp
)
target_link_libraries(
	exp_pcl_gicp 
	${LIBRARIES}
)

#------------------------------------------------------------------------------
#  exp_pcl_fpfhicp 
#------------------------------------------------------------------------------
add_executable(
	exp_pcl_fpfhicp 
	exp_pcl_fpfhicp.cpp
)
target_link_libraries(
	exp_pcl_fpfhicp 
	${LIBRARIES}
)