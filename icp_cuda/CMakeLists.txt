################################################################################
#
# Project: Iterative Closest Point with CUDA-Support
# Author:  Simon Schreiberhuber
#
################################################################################


# Prerequisites
cmake_minimum_required(VERSION 3.8)
if(NOT DEFINED CMAKE_CUDA_COMPILER)
	set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
endif()


# Project
project(icp_cuda
	LANGUAGES CXX CUDA
)

set(LIB_NAME ${PROJECT_NAME})

# Definitions
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_CUDA_FLAGS 
	"${CMAKE_CUDA_FLAGS} \
		--expt-relaxed-constexpr \
		--ftz=true \
		--prec-div=false \
		--prec-sqrt=false \
		 -Xcompiler -fPIC \
	"
)


# Required packages
find_package(CUDA REQUIRED)
find_package(Eigen3 3.3.5 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)


# Includes
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})


# Source files
file(GLOB CXX_SRCS 
	*.cpp 
)
file(GLOB CUDA_SRCS 
	cuda/*.cu
)
file(GLOB CUDA_CONTAINERS 
	cuda/containers/*.cpp
)


# Build
add_library(${PROJECT_NAME} SHARED
	${CXX_SRCS}
	${CUDA_SRCS}
	${CUDA_CONTAINERS}
)
target_link_libraries(${PROJECT_NAME}
	${Pangolin_LIBRARIES}
	${Eigen_LIBRARIES}
	${CUDA_LIBRARIES}
)


#---------------------------------------------------------------------#
#                            Installation                             #
#---------------------------------------------------------------------#


set(_include_install_dir "include/${LIB_NAME}")
## Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

## Install headers
install(
		DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
		DESTINATION ${_include_install_dir}
		FILES_MATCHING PATTERN "*.h"
)