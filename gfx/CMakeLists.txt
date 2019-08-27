cmake_minimum_required (VERSION 3.8)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
set(CUDA_INCLUDE_DIRS "/usr/local/cuda/include")
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 11)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr ")

project(gfx)



find_package(glfw3 3.1 REQUIRED)
find_package(GLEW REQUIRED)

if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    link_libraries(${GLEW_LIBRARIES})
    message("glew found")

    message("${GLEW_LIBRARIES}")
endif()
find_package(OpenGL REQUIRED)
find_package(OpenCV 3.4.3 REQUIRED)
#find_package(datasetLoader REQUIRED)



include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLFW_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
#message("${datasetLoader_SOURCE_DIR}")
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})


set(HEADERS
            shader.h
            camera.h
            view.h
            glUtils.h
            gpuTex.h)

set(SOURCES shader.cpp
            camera.cpp
            view.cpp
            glUtils.cpp
            gpuTex.cpp
        ../segmentation/IncrementalSegmentation.h GarbageCollector.cpp GarbageCollector.h)



#SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  -ftemplate-backtrace-limit=0; --cudart;  static; -O4; --expt-relaxed-constexpr; -fmad=true; --expt-extended-lambda;--relocatable-device-code=false ;
#-Xptxas -dlcm=ca; -Xptxas --opt-level=4 )
add_library(gfx ${HEADERS} ${SOURCES} ${SHADERS})
target_link_libraries(gfx ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw ${QT_LIBRARIES})

install (TARGETS gfx DESTINATION lib)
#add_definitions(-std=gnu++11)
#set(CMAKE_CXX_FLAGS  "-Wall -std=c++11")

