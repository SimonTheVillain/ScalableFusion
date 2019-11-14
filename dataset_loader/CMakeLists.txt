################################################################################
#
# Project: Tool for loading datasets
# Author:  Simon Schreiberhuber
#
################################################################################


# Prerequisites
cmake_minimum_required(VERSION 3.8)


# Project
project(dataset_loader
	LANGUAGES CXX
)

set(LIB_NAME ${PROJECT_NAME})

# Required packages
find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 4 REQUIRED)
endif()
find_package(radical CONFIG REQUIRED)


# Includes
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})


# Links
link_directories(${OpenCV_LIBRARY_DIRS})


# Source files
file(GLOB CXX_SRCS 
	*.cpp 
)


# Build
add_library(${PROJECT_NAME} SHARED
	${CXX_SRCS}
)
target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBRARIES}
	radical
)

#---------------------------------------------------------------------#
#                                 Tests                               #
#---------------------------------------------------------------------#
add_executable(${PROJECT_NAME}_test
		test/dataset_loader_test.cpp
		)
target_link_libraries(${PROJECT_NAME}_test
		${PROJECT_NAME}
		)
set_target_properties(${PROJECT_NAME}_test
		PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../test/"
		)



#---------------------------------------------------------------------#
#                            Installation                             #
#---------------------------------------------------------------------#



set(_config_install_dir "lib/cmake/${PROJECT_NAME}")
set(_include_install_dir "include/${LIB_NAME}")
set(_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

set(_version_config_file "${_generated_dir}/${PROJECT_NAME}-config-version.cmake")
set(_project_config_file "${_generated_dir}/${PROJECT_NAME}-config.cmake")
set(_targets_export_name "${PROJECT_NAME}-targets")

## Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

## Install headers
install(
		DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
		DESTINATION ${_include_install_dir}
		FILES_MATCHING PATTERN "*.h"
)

## Instal config files
#install(
#		FILES ${_project_config_file} ${_version_config_file}
#		DESTINATION "${_config_install_dir}"
#)
#
#install(
#		EXPORT "${_targets_export_name}"
#		DESTINATION "${_config_install_dir}"
#)


