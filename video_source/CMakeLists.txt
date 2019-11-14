################################################################################
#
# Project: Tool for accessing video sources
# Author:  Nikolaus Wagner
#
################################################################################


# Prerequisites
cmake_minimum_required(VERSION 3.8)


# Project
project(video_source
	LANGUAGES CXX
)
set(LIB_NAME ${PROJECT_NAME})



# Required packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 4 REQUIRED)
endif()
find_package(radical CONFIG REQUIRED)


# Includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${OpenCV_INCLUDE_DIRS})


# Links
link_directories(${OpenCV_LIBRARY_DIRS})


# Source files
file(GLOB CXX_SRCS 
	src/*
)


# Build
add_library(${PROJECT_NAME} SHARED
	${CXX_SRCS}
)
target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBRARIES}
	radical
)

# Tests
add_executable(${PROJECT_NAME}_test
		test/test_tuwdataset.cpp
		)
target_link_libraries(${PROJECT_NAME}_test
		${PROJECT_NAME}
		)

#---------------------------------------------------------------------#
#                            Installation                             #
#---------------------------------------------------------------------#

# Install
set(_include_install_dir "include/${LIB_NAME}")

## Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

## Install headers
install(
		DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
		DESTINATION ${_include_install_dir}
		FILES_MATCHING PATTERN "*.h"
)
