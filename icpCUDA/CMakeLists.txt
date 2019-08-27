cmake_minimum_required(VERSION 3.8)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
set(CUDA_INCLUDE_DIRS "/usr/local/cuda/include")
enable_language(CUDA)
set(CMAKE_CXX_STANDARD 11) #cpp 11 not 14
set(CMAKE_CUDA_STANDARD 11)

project(ICP)
#set(CMAKE_BUILD_TYPE DEBUG)

find_package(Pangolin REQUIRED)
message("${CMAKE_MODULE_PATH}")
#find_package(CUDA 10 REQUIRED)
find_package(Eigen3 3.3.5 REQUIRED)
find_package(Sophus REQUIRED)



      
include_directories(${Pangolin_INCLUDE_DIRS})
#message("${Pangolin_INCLUDE_DIRS}")#simon debug
include_directories(${CUDA_INCLUDE_DIRS})
#message("${CUDA_INCLUDE_DIRS}")#simon debug
include_directories(${EIGEN_INCLUDE_DIRS})
#message("${EIGEN_INCLUDE_DIRS}")#simon debug
include_directories(${Sophus_INCLUDE_DIR})
#message("${Sophus_INCLUDE_DIRS}")#simon debug
#message(${Pangolin_INCLUDE_DIRS})
#message(${CUDA_INCLUDE_DIRS})
#message(${EIGEN_INCLUDE_DIRS})
#message(${Sophus_INCLUDE_DIR})

file(GLOB srcs *.cpp)
file(GLOB cuda Cuda/*.cu)
file(GLOB containers Cuda/containers/*.cpp)

set(CUDA_ARCH_BIN "30 32 35 37 50 52 60 61" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
include(CudaComputeTargetFlags.cmake)                  
APPEND_TARGET_ARCH_FLAGS()

#string(APPEND CMAKE_CUDA_FLAGS " -gencode arch=compute_50,code=sm_50")


set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --expt-relaxed-constexpr")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler;-fPIC; -std=c++11")
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --ftz=true;--prec-div=false;--prec-sqrt=false")#;--expt-relaxed-constexpr") #the original
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --ftz=false;--prec-div=true;--prec-sqrt=true")#;--expt-relaxed-constexpr")

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -fPIC")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr ")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --ftz=true --prec-div=false --prec-sqrt=false")
#CUDA_COMPILE(cuda_objs ${cuda})

if(NOT (CMAKE_BUILD_TYPE MATCHES "DEBUG"))
    set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11")
endif(NOT (CMAKE_BUILD_TYPE MATCHES "DEBUG"))


#simons stupid workaround
add_executable(ICP 
               ${srcs}
               ${cuda} 
               #${cuda_objs}
               ${containers}
)

target_link_libraries(ICP
                      ${Pangolin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${CUDA_LIBRARIES}
)


add_library(ICPCUDA
        ICPOdometry.h
        ICPOdometry.cpp# dirty workaround
        ${cuda}
        ${cuda_objs}
        ${containers}
    )

#cuda_add_library(ICPCUDA
#    ICPOdometry.h
#    ICPOdometry.cpp# dirty workaround
#    ${cuda}
#    ${cuda_objs}
#    ${containers}
#)
target_link_libraries(ICPCUDA
                      ${Pangolin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${CUDA_LIBRARIES}
)
