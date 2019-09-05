################################################################################
#
# Project: Segmentation
# Author:  Simon Schreiberhuber
#
################################################################################


# Prerequisites
cmake_minimum_required (VERSION 3.8)


# Project
project(segmentation
	LANGUAGES CXX
)


# Definitions
set(CMAKE_CXX_STANDARD 11)


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
	${OpenCV_LIBS}
)


# Build
install (TARGETS ${PROJECT_NAME} DESTINATION lib)
