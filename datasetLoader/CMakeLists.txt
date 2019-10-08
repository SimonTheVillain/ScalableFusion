cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 11)

project (datasetLoader)
find_package(OpenCV 4 REQUIRED)



find_package(radical CONFIG REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

#message("${CMAKE_CURRENT_SOURCE_DIR}")
#set(datasetLoader_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)#gfx_SOURCE_DIR
#message("${datasetLoader_SOURCE_DIR}")

add_library(datasetLoader DatasetLoader.cpp DatasetLoader.h)

target_link_libraries(datasetLoader  boost_system boost_filesystem  ${OpenCV_LIBS} radical)

#set(CMAKE_CXX_FLAGS                "-Wall -std=c++11")
#set (CMAKE_CXX_STANDARD 11)
