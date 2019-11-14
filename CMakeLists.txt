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
#add_definitions(-DEIGEN_DONT_VECTORIZE)

# Project
project(scalable_fusion
	LANGUAGES CXX CUDA
)
set(LIB_NAME scalfu)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
string(TOLOWER ${CMAKE_BUILD_TYPE} LOWERCASE_CMAKE_BUILD_TYPE)
if(LOWERCASE_CMAKE_BUILD_TYPE MATCHES debug)
	add_definitions(-DDEBUG)
endif(LOWERCASE_CMAKE_BUILD_TYPE MATCHES debug)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O3 -march=native")#O3 should only be in release mode.
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native") #TODO: make everything runable with these optimization settings as well

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
			-O3 \
		"
			) #--device-debug #everything fails at device-debug
endif()


#define library version:
set(SCALFU_VERSION_MAJOR 0)
set(SCALFU_VERSION_MINOR 7)
set(SCALFU_VERSION_PATCH 0)
set(SCALFU_VERSION ${SCALFU_VERSION_MAJOR}.${SCALFU_VERSION_MINOR}.${SCALFU_VERSION_PATCH})
set(SCALFU_VERSION_SO "${SCALFU_VERSION_MAJOR}.${SCALFU_VERSION_MINOR}")
set(SCALFU_VERSION_LIB "${SCALFU_VERSION_MAJOR}.${SCALFU_VERSION_MINOR}.${SCALFU_VERSION_PATCH}")
# Defintions


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
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 3 REQUIRED)
	set(PREFERRED_OPENCV_VERSION 3)
else()
	set(PREFERRED_OPENCV_VERSION 4)
endif()
find_package(OpenGL REQUIRED)
find_package(radical CONFIG REQUIRED)
find_package(CUDA REQUIRED)

set(SCALFU_INCLUDE_DIRS
		"${CMAKE_SOURCE_DIR}/include"
		#"${CMAKE_SOURCE_DIR}/videl_source/include"
		${CMAKE_SOURCE_DIR}
		${CUDA_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}
		${GLEW_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
		)
set(SCALFU_LIBRARIES
		${LIB_NAME}
		glfw
		glog
		icp_cuda
		radical
		dataset_loader
		video_source
		${ASSIMP_LIBRARIES}
		${Boost_LIBRARIES}
		${OPENGL_gl_LIBRARY}
		${OPENGL_glu_LIBRARY}
		${OpenCV_LIBS})
set(SCALFU_LIBRARY_DIRS
		${ASSIMP_LIBRARY_DIRS}
		${GLEW_LIBRARY_DIRS}
		${OpenCV_LIBRARY_DIRS}
		${GLEW_LIBRARIES})


#list(APPEND scafu_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS})



# Add subprojects
add_subdirectory(dataset_loader)
add_subdirectory(video_source)
add_subdirectory(icp_cuda)
add_subdirectory(src)
add_subdirectory(applications)
add_subdirectory(tests)


# setup package file
#include(CMakePackageConfigHelpers)
#write_basic_package_version_file(
#		"${_version_config_file}"
#		VERSION ${SCALFU_VERSION}
#		COMPATIBILITY AnyNewerVersion
#)


#TODO: https://foonathan.net/2016/03/cmake-install/
#set(tool_dest "bin")
#set(include_dest "include/scalfu-${SCALFU_VERSION}")
#set(main_lib_dest "lib/scalfu-${SCALFU_VERSION}")

#---------------------------------------------------------------------#
#                            Uninstallation                             #
#---------------------------------------------------------------------#


if(NOT TARGET uninstall)
	configure_file(
			"${CMAKE_CURRENT_SOURCE_DIR}/cmake/Uninstall.cmake.in"
			"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
			IMMEDIATE @ONLY)

	add_custom_target(uninstall
			COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

