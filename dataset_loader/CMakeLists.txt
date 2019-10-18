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


# Required packages
find_package(OpenCV 3.4.4 QUIET)
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
add_library(${PROJECT_NAME} 
	${CXX_SRCS}
)
target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBRARIES}
	radical
)


# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)


# Tests
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