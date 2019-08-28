#------------------------------------------------------------------------------
#  Cmake version 
#------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.12.4 FATAL_ERROR)

#-------------------------------------------------------------------------------
#  Project
#-------------------------------------------------------------------------------
set(PROJ_NAME test-PCL)
project(${PROJ_NAME}
		LANGUAGES 		CXX
)

#-------------------------------------------------------------------------------
#  Set output path
#-------------------------------------------------------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY 		${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY 		${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY 		${PROJECT_BINARY_DIR}/lib)

#-------------------------------------------------------------------------------
#  Setup my library
#-------------------------------------------------------------------------------
include_directories(include)
aux_source_directory(src 	mypclSRCS)

#-------------------------------------------------------------------------------
#  Setup pcl library
#-------------------------------------------------------------------------------
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#-------------------------------------------------------------------------------
#  Link all library
#-------------------------------------------------------------------------------
set(LIBRARIES 	${PROJ_NAME})
add_library(
	${LIBRARIES} STATIC 
	${mypclSRCS}
)
target_link_libraries(
	${LIBRARIES}
	${PCL_LIBRARIES}
)

#-------------------------------------------------------------------------------
#  Add subdirectory
#-------------------------------------------------------------------------------
add_subdirectory(app)
