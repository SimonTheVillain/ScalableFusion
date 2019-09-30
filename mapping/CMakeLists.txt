################################################################################
#
# Project: Mapping
# Author:  Simon Schreiberhuber
#
################################################################################


# Prerequisites
cmake_minimum_required (VERSION 3.8)
if(NOT DEFINED CMAKE_CUDA_COMPILER)
	set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
endif()


# Project
project(mapping
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
if(CMAKE_BUILD_TYPE MATCHES DEBUG)
	set(CMAKE_CUDA_FLAGS 
		"${CMAKE_CUDA_FLAGS} \
			--generate-line-info \
		"
	) #--device-debug #everything fails at device-debug
endif()
set(CMAKE_CXX_FLAGS 
	"${CMAKE_CXX_FLAGS} \
		--std=c++11 \
	"
)


# Required packages
find_package(ASSIMP REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.1 REQUIRED)
find_package(OpenCV 3.4.4 QUIET)
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
include_directories(./base)
include_directories(./cuda)
include_directories(./export)
include_directories(./gpu)
include_directories(./remeshing)
include_directories(./rendering)
include_directories(./rendering/shader)
include_directories(./utils)
include_directories(./xtionCamModel)
include_directories(../datasetLoader)
include_directories(../gfx)
include_directories(../icpCUDA)


# Links
link_directories(${ASSIMP_LIBRARY_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries(${GLEW_LIBRARIES})


# Source files
file(GLOB CXX_SRCS
	base/meshStructure.cpp
	base/textureStructure.cpp
	cuda/geomUpdate.cpp
	export/exportMap.cpp
	gpu/ActiveSet.cpp 
	gpu/gpuBuffer.cpp
	gpu/gpuGeomStorage.cpp
	gpu/texAtlas.cpp
	graph/DeformationGraph.cpp
	graph/DeformationNode.cpp
	rendering/lowDetailMapRenderer.cpp
	rendering/mapInformationRenderer.cpp
	rendering/mapPresentationRenderer.cpp
	rendering/renderableModel.cpp
	utils/gpuNormSeg.cpp
	utils/perfMeter.cpp
	utils/principalPlane.cpp
	debugRender.cpp
	GeometryUpdate.cpp
	intermediateDepthModel.cpp
	Labelling.cpp
	Meshing.cpp
	meshReconstruction.cpp
	scheduler.cpp
	schedulerThreaded.cpp
	Stitching.cpp
	Texturing.cpp
	worker.cpp 
)
file(GLOB CUDA_SRCS
	cuda/coalescedMemoryTransfer.cu
	cuda/coarseUpdate.cu
	cuda/float16_utils.cu
	cuda/geomUpdate.cu
	cuda/labelling.cu
	cuda/normEst.cu
	cuda/reproject.cu
	cuda/stdTexUpdate.cu
	cuda/test.cu
	cuda/texCoords.cu
	cuda/texPatchInit.cu
	cuda/vertexUpdate.cu
	cuda/xtionCameraModel.cu
)
file(GLOB SHADERS
	rendering/shader/*.frag
	rendering/shader/*.geom
	rendering/shader/*.vert
)


# Build
add_executable(${PROJECT_NAME}
	main.cpp
	${CXX_SRCS}
	${CUDA_SRCS} 
	${SHADERS}
)
target_link_libraries(${PROJECT_NAME}
	${ASSIMP_LIBRARIES} 
	${Boost_LIBRARIES}
	datasetLoader 
	glfw 
	glog 
	gfx 
	icpCUDA 
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY} 
	${OpenCV_LIBS} 
	radical
	segmentation
)


# Tests
add_executable(textureMemTest 
	tests/TextureMemTest.cpp 
	${CXX_SRCS} 
	${CUDA_SRCS} 
	${SHADERS}
)
target_link_libraries(textureMemTest 
	${ASSIMP_LIBRARIES} 
	${Boost_LIBRARIES} 
	datasetLoader 
	glfw 
	glog
	gfx 
	icpCUDA 
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY} 
	${OpenCV_LIBS} 
	segmentation
)

add_executable(textureMemTestThreaded 
	tests/TextureMemTestThreaded.cpp 
	${CXX_SRCS} 
	${CUDA_SRCS} 
	${SHADERS}
)
target_link_libraries(textureMemTestThreaded 
	${ASSIMP_LIBRARIES} 
	${Boost_LIBRARIES} 
	datasetLoader 
	glfw 
	glog
	gfx 
	icpCUDA 
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY} 
	${OpenCV_LIBS} 
	segmentation
)

add_executable(GraphDeformationTest 
	tests/GraphDeformationTest.cpp 
	${CXX_SRCS} 
	${CUDA_SRCS} 
	${SHADERS}
	base/stackVector.h 
	graph/DeformationGraph.cpp
	graph/DeformationGraph.h 
	graph/DeformationNode.cpp 
	graph/DeformationNode.h
)
target_link_libraries(GraphDeformationTest 
	${ASSIMP_LIBRARIES} 
	${Boost_LIBRARIES} 
	datasetLoader 
	glfw 
	glog
	gfx 
	icpCUDA 
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY} 
	${OpenCV_LIBS} 
	segmentation
)

add_executable(LabelCloudViewer 
	tests/LabelCloudViewer.cpp 
	${CXX_SRCS} 
	${CUDA_SRCS} 
	${SHADERS}
	base/stackVector.h 
	graph/DeformationGraph.cpp 
	graph/DeformationGraph.h 
	graph/DeformationNode.cpp 
	graph/DeformationNode.h
)
target_link_libraries(LabelCloudViewer 
	${ASSIMP_LIBRARIES} 
	${Boost_LIBRARIES} 
	datasetLoader 
	glfw 
	glog
	gfx 
	icpCUDA 
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY} 
	${OpenCV_LIBS} 
	segmentation
)

add_executable(SimpleCloudViewer 
	tests/SimpleCloudViewer.cpp 
	${CXX_SRCS} 
	${CUDA_SRCS} 
	${SHADERS}
)
target_link_libraries(SimpleCloudViewer 
	${ASSIMP_LIBRARIES} 
	${Boost_LIBRARIES} 
	datasetLoader 
	glfw 
	glog
	gfx 
	icpCUDA 
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY} 
	${OpenCV_LIBS} 
	segmentation
)


# Notes

#for better cuda support 3.8.2 would be nice!!!!! (wait for ubuntu 18.04)
#https://stackoverflow.com/questions/36551469/triggering-c11-support-in-nvcc-with-cmake

#vertex decimation.... somewhere page 35
#ask how to do it if hole will not be convex
#idea.... see all edge points of this hole and make it convex. by shifting the points to the convex hull
#if only one point destroys the convexity this one could be the point for every other to connect to (half edge collapse)
#problem is, that edges won't be of similar length ->bad
#http://graphics.stanford.edu/courses/cs468-10-fall/LectureSlides/08_Simplification.pdf


#Note on adding new points to patches:
#only do it on border. points on top that are too far apart should not be added.
#mark the border by drawing border pixel of the patches