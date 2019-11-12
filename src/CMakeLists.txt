################################################################################
#
# Project: Mapping
# Author:  Simon Schreiberhuber
#
################################################################################


# Prerequisites (SIMON: not really though)
#cmake_minimum_required (VERSION 3.8)
#if(NOT DEFINED CMAKE_CUDA_COMPILER)
#	set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
#endif()


# Project
#project(scalable_fusion
#	LANGUAGES CXX CUDA
#)


# Definitions
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_CUDA_FLAGS 
	"${CMAKE_CUDA_FLAGS} \
		--expt-relaxed-constexpr \
	"
)
if(CMAKE_BUILD_TYPE MATCHES DEBUG)
	set(CMAKE_CUDA_FLAGS 
		"${CMAKE_CUDA_FLAGS} \
			--generate-line-info \
		"
	) #--device-debug #everything fails at device-debug
endif()

## this is necessary? isn't there set(CMAKE_CXX_STANDARD 11) up there?
#set(CMAKE_CXX_FLAGS
#	"${CMAKE_CXX_FLAGS} \
#		--std=c++11 \
#	"
#)


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


# Includes
include_directories(${ASSIMP_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR})#for icp_cuda and dataset_loader

# Links
link_directories(${ASSIMP_LIBRARY_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries(${GLEW_LIBRARIES})


# Source files
file(GLOB CXX_SRCS
		base/mesh_structure.cpp
		base/texture_structure.cpp
		cuda/geom_update.cpp
		export/map_exporter.cpp
		gpu/active_set.cpp
		gpu/gpu_buffer.cpp
		gpu/gpu_geom_storage.cpp
		gpu/tex_atlas.cpp
		graph/deformation_graph.cpp
		graph/deformation_node.cpp
		rendering/low_detail_renderer.cpp
		rendering/information_renderer.cpp
		rendering/presentation_renderer.cpp
		rendering/renderable_model.cpp
		utils/gpu_norm_seg.cpp
		utils/perf_meter.cpp
		utils/principal_plane.cpp
		debug_render.cpp
		geometry_updater.cpp
		intermediate_depth_model.cpp
		labelling.cpp
		mesher.cpp
		mesh_reconstruction.cpp
		scheduler.cpp
		scheduler_threaded.cpp
		mesh_stitcher.cpp
		texture_updater.cpp
		worker.cpp
)
file(GLOB CUDA_SRCS
		cuda/coalesced_memory_transfer.cu
		cuda/coarse_update.cu
		cuda/float16_utils.cu
		cuda/geom_update.cu
		cuda/labelling_cuda.cu
		cuda/norm_est.cu
		cuda/reproject.cu
		cuda/std_tex_update.cu
		cuda/test.cu
		cuda/tex_coords.cu
		cuda/tex_patch_init.cu
		cuda/vertex_update.cu
		cuda/xtion_camera_model.cu
)
file(GLOB SHADERS
		rendering/shader/*.frag
		rendering/shader/*.geom
		rendering/shader/*.vert
)



add_library(scalable_fusion
		${CXX_SRCS}
		${CUDA_SRCS}
		${SHADERS})


# Notes

#for better cuda support 3.8.2 would be nice!!!!! (wait for ubuntu 18.04)
#https://stackoverflow.com/questions/36551469/triggering-c11-support-in-nvcc-with-cmake

#vertex decimation.... somewhere page 35
#ask how to do it if hole will not be convex
#idea.... see all edge points of this hole and make it convex. by shifting the points to the convex hull
#if only one point destroys the convexity this one could be the point for every other to connect to (half edge collapse)
#problem is, that edges won't be of similar length ->bad
#http://graphics.stanford.edu/courses/cs468-10-fall/LectureSlides/08_Simplification.pdf


#Note on adding new points to patches:
#only do it on border. points on top that are too far apart should not be added.
#mark the border by drawing border pixel of the patches