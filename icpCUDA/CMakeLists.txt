################################################################################
#
# Project: Iterative Closest Point with CUDA-Support
# Author:  Simon Schreiberhuber
#
################################################################################
cmake_minimum_required(VERSION 3.8)
project(icpCUDA
	LANGUAGES CXX CUDA
)


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
	Cuda/*.cu
)
file(GLOB CUDA_CONTAINERS 
	Cuda/containers/*.cpp
)


# Build
add_library(${PROJECT_NAME}
	${CXX_SRCS}
	${CUDA_SRCS}
	${CUDA_CONTAINERS}
)
target_link_libraries(${PROJECT_NAME}
	${Pangolin_LIBRARIES}
	${Eigen_LIBRARIES}
	${CUDA_LIBRARIES}
)


# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)