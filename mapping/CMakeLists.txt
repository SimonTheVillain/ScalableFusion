cmake_minimum_required (VERSION 3.8)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/")
enable_language(CUDA)
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--expt-relaxed-constexpr")

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr ")
if(CMAKE_BUILD_TYPE MATCHES DEBUG)
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --generate-line-info") #--device-debug #everything fails at device-debug
endif()


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CUDA_STANDARD 11)
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

project(mapping)

#set (CMAKE_CXX_FLAGS "--std=c++11")
#set (CUDA_PROPAGATE_HOST_FLAGS ON)
#set(CUDA_INCLUDE_DIRS /usr/local/cuda/include/ ${CUDA_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Release)

#DESPARATE MEASURES
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

find_package(glfw3 3.1 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV 3.4.3 REQUIRED)
find_package(ASSIMP REQUIRED)

#find_package(PCL 1.2 REQUIRED)

FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )


#set(CMAKE_MODULE_PATH /home/simon/libraries/eigen-3.3.3/cmake)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
message("I Hate you CMAKE.... do you hear that? EIGEN!!! YOU ARE NOT ANY BETTER!!!!")
message(${EIGEN3_INCLUDE_DIR})


#find_package(datasetLoader REQUIRED)
#find_package(gfx REQUIRED)
#set(ICP_DIR "/home/simon/libraries/ICPCUDA-master/src/build")
#find_package(ICP REQUIRED)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")


#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLFW_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ASSIMP_INCLUDE_DIRS})

#TODO: these two should be loaded as include directory
#(only works because they share source and include directory)
include_directories(${datasetLoader_SOURCE_DIR})
include_directories(${gfx_SOURCE_DIR})
include_directories(${ICPCUDA_SOURCE_DIR})

include_directories(
    base
    cuda
    export
    gpu
    remeshing
    rendering
    rendering/shader
    utils
    xtionCamModel)


#find_package(gfx REQUIRED)
find_library(gfx_LIBRARY
  NAMES gfx
  PATHS "${CMAKE_SOURCE_DIRS}/gfx"
)

include_directories(${gfx_INCLUDE_DIR})
message("I Hate you CMAKE.... do you hear that?")
message("${gfx_INCLUDE_DIRS}")
message("${gfx_SOURCE_DIR}")
message("${ICPCUDA_INCLUDE_DIR}")
message("${ICPCUDA_SOURCE_DIR}")
#link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})



#https://github.com/PointCloudLibrary/pcl/issues/776



#find_package(PCL 1.7 REQUIRED COMPONENTS common io)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


#find_package(V4R REQUIRED)
#include_directories(${V4R_INCLUDE_DIRS})
#link_directories(${V4R_LIBRARY_DIRS})
#add_definitions(${V4R_DEFINITIONS})

set(HEADERS #cpuMeshStructure.h
        meshReconstruction.h
            rendering/mapInformationRenderer.h
            rendering/mapPresentationRenderer.h
            base/meshStructure.h
            base/textureStructure.h
            utils/arcball.h
            gpu/gpuGeomStorage.h
            utils/principalPlane.h
            utils/gpuNormSeg.h
            gpu/texAtlas.h
        StitchingUtils.h
            cuda/test.h
            cuda/texPatchInit.h
            cuda/stdTexUpdate.h
            cuda/xtionCameraModel.h
            cuda/geomUpdate.h
            cuda/gpuMeshStructure.h
            cuda/helper_math.h
            cuda/normSeg.h
            cuda/normEst.h
            cuda/vertexUpdate.h
            cuda/gpuErrchk.h
            cuda/surfaceRead.h
            cuda/coarseUpdate.h
            cuda/float16_utils.h
            cuda/reproject.h
            cuda/coalescedMemoryTransfer.h
            cuda/texCoords.h
            utils/octree.h
            rendering/renderableModel.h
            rendering/lowDetailMapRenderer.h
            scheduler.h
            gpu/threadSafeFBO_VAO.h
            intermediateDepthModel.h
            gpu/gpuBuffer.h
            utils/perfMeter.h
            utils/gpuPresets.h
            xtionCamModel/xtionModel.h
            export/exportMap.h
            #remeshing/remeshing.h
            #remeshing/patchClustering.h
            cuda/labelling.h
            #segmentation/segmentation.h
            debugRender.h
        worker.h gpu/ActiveSet.h schedulerThreaded.h)
set(SOURCES
        meshReconstruction.cpp
            rendering/mapInformationRenderer.cpp
            rendering/mapPresentationRenderer.cpp
            base/meshStructure.cpp
            base/textureStructure.cpp
        utils/principalPlane.cpp
            gpu/gpuGeomStorage.cpp
            utils/gpuNormSeg.cpp
            gpu/texAtlas.cpp
            cuda/test.cu
            cuda/texPatchInit.cu
            cuda/stdTexUpdate.cu
            cuda/xtionCameraModel.cu
            cuda/geomUpdate.cu
            cuda/geomUpdate.cpp
            cuda/normSeg.cu
            cuda/normEst.cu
            cuda/vertexUpdate.cu
            cuda/coarseUpdate.cu
            cuda/float16_utils.cu
            cuda/reproject.cu
            cuda/coalescedMemoryTransfer.cu
            cuda/texCoords.cu
            rendering/renderableModel.cpp
            rendering/lowDetailMapRenderer.cpp
            scheduler.cpp
            intermediateDepthModel.cpp
            gpu/gpuBuffer.cpp
            utils/perfMeter.cpp
            utils/gpuPresets.cpp
        xtionCamModel/xtionModel.cpp
            export/exportMap.cpp
            #remeshing/remeshing.cpp
            #remeshing/patchClustering.cpp
            #segmentation/segmentation.cpp
            cuda/labelling.cu
            debugRender.cpp
        graph/DeformationGraph.cpp
        graph/DeformationGraph.h
        graph/DeformationNode.cpp
        graph/DeformationNode.h
        Stitching.cpp
        Stitching.h
        Labelling.cpp
        Labelling.h
        GeometryUpdate.cpp
        GeometryUpdate.h
        Meshing.cpp
        Meshing.h
        Texturing.cpp
        Texturing.h
        worker.cpp gpu/ActiveSet.cpp schedulerThreaded.cpp)

set(SHADERS #shader/passtrough.vert
            rendering/shader/presentation.frag
            rendering/shader/presentation.vert
            rendering/shader/information.frag
            rendering/shader/information.vert
            rendering/shader/coordinates.frag
            rendering/shader/coordinates.vert
            rendering/shader/coordinatesNoTriangle.frag
            rendering/shader/coordinatesNoTriangle.vert
            rendering/shader/presentationDebug.frag
            rendering/shader/presentationDebug.vert
            rendering/shader/datastructure.glsl
            rendering/shader/unlit.vert
            rendering/shader/unlit.frag
            rendering/shader/lowDetail.vert
            rendering/shader/lowDetail.geom
            rendering/shader/lowDetail.frag
            rendering/shader/lowDetailRefDepth.frag
            rendering/shader/lowDetailRefDepth.geom
            rendering/shader/lowDetailRefDepth.vert
            rendering/shader/debug.frag
            rendering/shader/debug.vert
            rendering/shader/triangleRefDepth.frag
            rendering/shader/triangleRefDepth.geom
            rendering/shader/triangleRefDepth.vert
            rendering/shader/debugGeometry.frag
            rendering/shader/debugGeometry.vert
            )


#this down here is just a test! for some reason this doesn't work
#find_package(V4R REQUIRED)
#set(CUDA_PROPAGATE_HOST_FLAGS ON)
#include_directories(${V4R_INCLUDE_DIRS})
#link_directories(${V4R_LIBRARY_DIRS})
#add_definitions(${V4R_DEFINITIONS})



#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC; -std=c++11")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--expt-relaxed-constexpr")


add_executable(mapping main.cpp ${HEADERS} ${SOURCES} ${SHADERS})
#set_property(TARGET mapping PROPERTY CUDA_STANDARD 11)
target_link_libraries(mapping ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw gfx ICPCUDA datasetLoader segmentation
        ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} glog) #until here its all wekk and actua
        #boost_system boost_filesystem  ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} v4r_core v4r_common v4r_attention_segmentation)# v4r_core v4r_common v4r_attention_segmentation) )
#boost_system boost_filesystem  ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} v4r_core v4r_common v4r_attention_segmentation
#target_link_libraries(mapping ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw ${QT_LIBRARIES}
#    datasetLoader gfx ICPCUDA ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} segmentation)


add_executable(textureMemTest tests/TextureMemTest.cpp ${HEADERS} ${SOURCES} ${SHADERS})
target_link_libraries(textureMemTest ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw gfx ICPCUDA datasetLoader segmentation
        ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} glog) #until here its all wekk and actua


add_executable(textureMemTestThreaded tests/TextureMemTestThreaded.cpp ${HEADERS} ${SOURCES} ${SHADERS})
target_link_libraries(textureMemTestThreaded ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw gfx ICPCUDA datasetLoader segmentation
        ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} glog) #until here its all wekk and actua

add_executable(GraphDeformationTest tests/GraphDeformationTest.cpp ${HEADERS} ${SOURCES} ${SHADERS} graph/DeformationGraph.cpp graph/DeformationGraph.h base/stackVector.h graph/DeformationNode.cpp graph/DeformationNode.h)
target_link_libraries(GraphDeformationTest ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw gfx ICPCUDA datasetLoader segmentation
        ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} glog) #until here its all wekk and actua


add_executable(LabelCloudViewer tests/LabelCloudViewer.cpp ${HEADERS} ${SOURCES} ${SHADERS} graph/DeformationGraph.cpp graph/DeformationGraph.h base/stackVector.h graph/DeformationNode.cpp graph/DeformationNode.h)
target_link_libraries(LabelCloudViewer ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw gfx ICPCUDA datasetLoader segmentation
        ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} glog) #until here its all wekk and actua


add_executable(SimpleCloudViewer tests/SimpleCloudViewer.cpp ${HEADERS} ${SOURCES} ${SHADERS})
target_link_libraries(SimpleCloudViewer ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw gfx ICPCUDA datasetLoader segmentation
        ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} glog) #until here its all wekk and actua