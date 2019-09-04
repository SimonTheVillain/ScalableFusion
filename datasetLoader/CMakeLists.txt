################################################################################
#
# Project: Tool for loading datasets
# Author:  Simon Schreiberhuber
#
################################################################################
cmake_minimum_required(VERSION 3.8)
project(datasetLoader
	LANGUAGES CXX
)


# Required packages
find_package(OpenCV 3.4.4 QUIET)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 4 REQUIRED)
endif()


# Includes
include_directories(${OpenCV_INCLUDE_DIRS})


# Links
link_directories(${OpenCV_LIBRARY_DIRS})


# Source files
file(GLOB CXX_SRCS 
	*.cpp 
)


# Build
add_library(${PROJECT_NAME} 
	${CXX_SRCS}
)
target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBRARIES}
)


# Install
install (TARGETS ${PROJECT_NAME} DESTINATION lib)
