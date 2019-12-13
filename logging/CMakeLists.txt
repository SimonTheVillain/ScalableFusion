################################################################################
#
# Project: Tool for creating log files
# Author:  Nikolaus Wagner
#
################################################################################


# Prerequisites
cmake_minimum_required(VERSION 3.8)


# Project
project(logging
	LANGUAGES CXX
)
set(LIB_NAME ${PROJECT_NAME})


# Includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)


# Tests
add_executable(${PROJECT_NAME}_test
	test/test_logging.cpp
)


#---------------------------------------------------------------------#
#                            Installation                             #
#---------------------------------------------------------------------#

# Install
set(_include_install_dir "include/${LIB_NAME}")


# Install headers
install(
		DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
		DESTINATION ${_include_install_dir}
		FILES_MATCHING PATTERN "*.h"
)
