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

# Required packages
find_package(ASSIMP REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.1 REQUIRED)
find_package(OpenCV 3.4.4 QUIET)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 4 REQUIRED)
endif()
find_package(OpenGL REQUIRED)
find_package(radical CONFIG REQUIRED)
find_package(CUDA REQUIRED)

list(APPEND SCALFU_INCLUDE_DIRS
		${CMAKE_SOURCE_DIR}
		${CUDA_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}
		${GLEW_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
		)
list(APPEND SCALFU_LIBRARIES
		scalable_fusion
		glfw
		gfx
		glog
		gfx
		icp_cuda
		radical
		segmentation
		dataset_loader
		${ASSIMP_LIBRARIES}
		${Boost_LIBRARIES}
		${OPENGL_gl_LIBRARY}
		${OPENGL_glu_LIBRARY}
		${OpenCV_LIBS})
list(APPEND SCALFU_LIBRARY_DIRS
		${ASSIMP_LIBRARY_DIRS}
		${GLEW_LIBRARY_DIRS}
		${OpenCV_LIBRARY_DIRS}
		${GLEW_LIBRARIES})


#list(APPEND scafu_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS})




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
add_subdirectory(applications)
add_subdirectory(tests)
