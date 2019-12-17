################################################################################
#
# Project: Mapping
# Author:  Simon Schreiberhuber
#
################################################################################




# Required packages
find_package(ASSIMP REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
set(Boost_VERSION_STRING "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}")
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.1 REQUIRED)
find_package(OpenCV ${PREFERRED_OPENCV_VERSION} QUIET)
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
include_directories(${CMAKE_SOURCE_DIR}/include/gfx)
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
		utils/octree.cpp
		utils/perf_meter.cpp
		utils/principal_plane.cpp
		segmentation/incremental_segmentation.cpp
		gfx/gl_utils.cpp
		gfx/garbage_collector.cpp
		gfx/camera.cpp
		gfx/gpu_tex.cpp
		gfx/shader.cpp
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



add_library(${LIB_NAME} SHARED
		${CXX_SRCS}
		${CUDA_SRCS}
		${SHADERS})


# Notes


#vertex decimation.... somewhere page 35
#ask how to do it if hole will not be convex
#idea.... see all edge points of this hole and make it convex. by shifting the points to the convex hull
#if only one point destroys the convexity this one could be the point for every other to connect to (half edge collapse)
#problem is, that edges won't be of similar length ->bad
#http://graphics.stanford.edu/courses/cs468-10-fall/LectureSlides/08_Simplification.pdf



#---------------------------------------------------------------------#
#                            Installation                             #
#---------------------------------------------------------------------#

## Layout:
#
#   <prefix>
#   ├── include
#   │   ├── base
#	|	├── cuda
#	|	├── export
#   └── lib
#       └── cmake
#           └── scalable_fusion

set(_config_install_dir "lib/cmake/${PROJECT_NAME}")
set(_include_install_dir "include/${LIB_NAME}")
set(_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

set(_version_config_file "${_generated_dir}/${PROJECT_NAME}-config-version.cmake")
set(_project_config_file "${_generated_dir}/${PROJECT_NAME}-config.cmake")
set(_targets_export_name "${PROJECT_NAME}-targets")

## Create scalfu-config-version.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
		"${_version_config_file}"
		VERSION ${SCALFU_VERSION}
		COMPATIBILITY AnyNewerVersion
)

## Create scalfu-config.cmake

configure_package_config_file(
		"../cmake/Config.cmake.in"
		"${_project_config_file}"
		INSTALL_DESTINATION "${_config_install_dir}"
)

## Install targets

install(TARGETS ${LIB_NAME}
		EXPORT "${_targets_export_name}"
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION ${_include_install_dir}
		)

## Install headers

install(
		DIRECTORY ${CMAKE_SOURCE_DIR}/include/
		DESTINATION ${_include_install_dir}
		FILES_MATCHING PATTERN "*.h"
)

## Instal config files

install(
		FILES ${_project_config_file} ${_version_config_file}
		DESTINATION "${_config_install_dir}"
)

install(
		EXPORT "${_targets_export_name}"
		DESTINATION "${_config_install_dir}"
)

#---------------------------------------------------------------------#
#                           Uninstallation                            #
#---------------------------------------------------------------------#

## Uninstall target

#configure_file(
#		"../cmake/Uninstall.cmake.in"
#		"${_generated_dir}/uninstall.cmake"
#		IMMEDIATE @ONLY
#)
#
#add_custom_target(uninstall
#		COMMAND ${CMAKE_COMMAND} -P ${_generated_dir}/uninstall.cmake
#		)
