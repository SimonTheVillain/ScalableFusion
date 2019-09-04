################################################################################
#
# Project: GFX
# Author:  Simon Schreiberhuber
#
################################################################################
cmake_minimum_required (VERSION 3.8)
project(gfx
	LANGUAGES CXX CUDA
)


# Definitions
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_CUDA_FLAGS 
	"${CMAKE_CUDA_FLAGS} \
		--expt-relaxed-constexpr \
	"
)


# Required packages
find_package(CUDA REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.1 REQUIRED)
find_package(OpenCV 3.4.4 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
endif()
find_package(OpenGL REQUIRED)


# Includes
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


# Links
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})


# Source files
file(GLOB CXX_SRCS 
	*.cpp
)


# Build
add_library(${PROJECT_NAME} 
	${CXX_SRCS}
)
target_link_libraries(${PROJECT_NAME} 
	glfw
	${OpenCV_LIBS}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}
	${QT_LIBRARIES}
)


# Install
install (TARGETS ${PROJECT_NAME} DESTINATION lib)
