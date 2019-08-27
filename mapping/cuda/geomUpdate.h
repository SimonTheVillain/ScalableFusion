#ifndef FILE_GEOM_UPDATE
#define FILE_GEOM_UPDATE


#include <cuda.h>
#include <cublas.h>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <memory>

#include "gpuMeshStructure.h"
#include "gpuErrchk.h"


/*
struct GeomUpdateDescriptor{


    int triangleSlot;//source of the triangles or are we really doing it with triangles
    int triangleCount;//target of the

    //actually: the preferrable solution:


    //Slot to a buffer filled with texture coordinates and references to vertices
    int texCoordSlot;
    int texCoordCount;

    cudaSurfaceObject_t stdTexSurf;//don't know if we want this
    cudaTextureObject_t stdTex;//or this
    cv::Rect2i stdTexRect;


    //every patch should be double buffered, in the GpuPatchInfo structure there should
    // be a field to where the current information is residing,
    //as soon as this is done, the variable within GpuPatchInfo is set to the new value.
    //(this also might be done with a kernel call)




};

*/

struct Vertex;
class MeshPatch;


namespace gpu{

    struct TexAtlasSubSurface{
        cudaSurfaceObject_t surface;
        cv::Rect2i rect;
    };


    struct UpdateDescriptor{
        cv::Rect2i destination;
        cv::Rect2f destinationN;
        cv::Size2i destinationSize;
        cudaSurfaceObject_t destinationReferences;

        cv::Point2i referenceOffset;

        cudaSurfaceObject_t destinationGeometry;

        cv::Rect2i source;
        cv::Rect2f sourceN;
        cv::Size2i sourceSize;
        cudaSurfaceObject_t sourceGeometry; // const in a sense of we wont overwrite its pixel
                                                //we might have to change that to be able to build
                                                //the structure
        //Pointers or indices to triangles are missing for this part.

        int vertexSourceStartInd;
        int vertexDestinationStartInd;
        int vertexCount;

        int patchInfoSlot;
        int triangleSlot;
        int triangleCount;


        bool updateTexture=false;//when all the neighbours are updloaded we also update the texture

    };



    //TODO: get rid of this!!!!!
    int updateGeometry(const cudaSurfaceObject_t geometryInput, //the sensor input adapted by standard deviations
                        int width, int height, //sensor resolution
                        const std::vector<UpdateDescriptor> &descriptors,
                        Eigen::Vector4f camPos,
                        Eigen::Matrix4f _pose, // because we want the vertex position relative to the camera
                        Eigen::Matrix4f proj_pose, //to get the position of the point on the image.
                        GpuVertex *vertices, Eigen::Vector2f *texPos,
                        GpuTriangle* triangles, GpuPatchInfo* patchInfos);



    class GeometryUpdate{
    public:
        struct CalcCenterTask{
            GpuVertex* vertices;
            uint32_t count;
        };

        static void calcCenterAndRadius(std::vector<std::shared_ptr<MeshPatch>> &patches);
        //std::vector<Eigen::Vector4f> (old return parameter) now it is directly updating the patches
    private:
        static void calcCenterAndRadiusKernelCall(dim3 grid, dim3 block,size_t bytes,
                CalcCenterTask * gpuTasks,Eigen::Vector4f * results);
    };



    class GeometryValidityChecks{
    public:
        struct VertexTask{
            uint32_t startSource;
            uint32_t startDest;
            uint32_t size;
        };

        static void checkVertexValidity(const cudaSurfaceObject_t sensor,
                             int width, int height,
                             Eigen::Matrix4f _pose,
                             Eigen::Matrix4f proj_pose,
                             std::vector<VertexTask> tasks,
                                 GpuVertex *vertices);

        struct TriangleTask{
            cudaSurfaceObject_t lookupTex;
            cv::Rect2i rect;
        };
        static void checkTriangleValidity(std::vector<TriangleTask> tasks,
                                          const cudaSurfaceObject_t sensor,
                                          int width,int height,
                                          Eigen::Matrix4f _pose, // because we want the vertex position relative to the camera
                                          Eigen::Matrix4f proj_pose, //to get the position of the point on the image.
                                          GpuVertex *vertices,
                                          Eigen::Vector2f *texPos,
                                          GpuTriangle* triangles,
                                          GpuPatchInfo* patchInfos);
    };


}


#endif

