cmake_minimum_required (VERSION 3.8)
#should not be necessary:
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-10.1")
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-10.1/bin/")


set (CMAKE_CXX_STANDARD 11)
project (segmentation)
#find_package(OpenCV 3.1 REQUIRED)
find_package(OpenCV 4 REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

#message("${CMAKE_CURRENT_SOURCE_DIR}")
#set(datasetLoader_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)#gfx_SOURCE_DIR
#message("${datasetLoader_SOURCE_DIR}")



#find_package(PCL 1.7 REQUIRED COMPONENTS common io)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


#This might help with this nasty fPIC error
#https://gitlab.kitware.com/cmake/cmake/issues/16915




#remove all this TUW stuff
#find_package(V4R REQUIRED)
#include_directories(${V4R_INCLUDE_DIRS})
#link_directories(${V4R_LIBRARY_DIRS})
#add_definitions(${V4R_DEFINITIONS})

#add_library(segmentation segmentation.cpp segmentation.h)

#target_link_libraries(segmentation  boost_system boost_filesystem  ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES}
#                        ${PCL_IO_LIBRARIES} v4r_core v4r_common v4r_attention_segmentation)

add_library(segmentation IncrementalSegmentation.h IncrementalSegmentation.cpp)
target_link_libraries(segmentation ${OpenCV_LIBS})


#set(CMAKE_CXX_FLAGS                "-Wall -std=c++11")
#set (CMAKE_CXX_STANDARD 11)
