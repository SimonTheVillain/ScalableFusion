################################################################################
#
# Project: Scaleable Fusion
# Author:  Simon Schreiberhuber
#
################################################################################


# Prerequisites
cmake_minimum_required (VERSION 3.8)
if(NOT DEFINED CMAKE_CUDA_COMPILER)
	set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
endif()
add_definitions(-DEIGEN_DONT_VECTORIZE)

# Project
project(ScalableFusion
	LANGUAGES CXX CUDA
)


# Defintions
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
string(TOLOWER ${CMAKE_BUILD_TYPE} LOWERCASE_CMAKE_BUILD_TYPE)
if(LOWERCASE_CMAKE_BUILD_TYPE MATCHES debug)
	add_definitions(-DDEBUG)
endif(LOWERCASE_CMAKE_BUILD_TYPE MATCHES debug)

#Don't throw away unused textures from the texture atlas,
#this is a speedup but could also mean a increased memory usage most of the time.
#TODO: if the memory leak is fixed rename this to RETAIN_ATLAS_TEXTURES_INDEFINITELY
add_definitions(-DGL_MEMORY_LEAK_WORKAROUND)

add_definitions(-DLOG_FRAMERATE)

#TODO: everything other than this is deprecated:!!!!
#therefore we should get rid of this definition
add_definitions(-DUSE_FLOAT16_GEOM_TEX)
#add_definitions(-DSHOW_SERIOUS_DEBUG_OUTPUTS)
add_definitions(-DPROPHYLACTIC_GPU_SYNC)#TODO: Implement this

add_definitions(-DIGNORE_SERIOUS_BUG_1)
#add_definitions(-DIGNORE_SERIOUS_BUG_2)
add_definitions(-DIGNORE_SERIOUS_BUG_3)
#add_definitions(-DIGNORE_SERIOUS_BUG_4)
#add_definitions(-DIGNORE_SERIOUS_BUG_5)//invalid stitches


# Includes
include_directories(${CMAKE_SOURCE_DIR}/dataset_loader/)
include_directories(${CMAKE_SOURCE_DIR}/gfx/)
include_directories(${CMAKE_SOURCE_DIR}/icp_cuda/)
include_directories(${CMAKE_SOURCE_DIR}/mapping/)
include_directories(${CMAKE_SOURCE_DIR}/segmentation/)


# Add subprojects
add_subdirectory(dataset_loader)
add_subdirectory(gfx)
add_subdirectory(icp_cuda)
add_subdirectory(mapping)
add_subdirectory(segmentation)
