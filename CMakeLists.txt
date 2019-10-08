cmake_minimum_required (VERSION 3.8)

#should not be necessary
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/")
include_directories("/usr/local/cuda/include/")


set(CMAKE_CXX_STANDARD 11)

project(supermapping)

set(CMAKE_BUILD_TYPE Debug)#Debug #Release





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


add_subdirectory(segmentation)
add_subdirectory(datasetLoader)
#message("${datasetLoader_SOURCE_DIR}")


add_subdirectory(gfx)
add_subdirectory(mapping)
add_subdirectory(icpCUDA)

