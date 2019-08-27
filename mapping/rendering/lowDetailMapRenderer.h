#ifndef FILE_LOW_DETAIL_MAP_RENDERER
#define FILE_LOW_DETAIL_MAP_RENDERER

#include <glUtils.h>
#include <shader.h>
#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <mutex>

#include <cuda.h>
#include <cublas.h>

#include "gpuMeshStructure.h"

class MeshPatch;
struct DoubleStitch;
struct TripleStitch;
class ActiveSet;



template<typename T>
struct GlCudaBuffer{
    GLuint glName;
    //todo: cuda resource
    size_t nrElements;
    cudaGraphicsResource_t cudaResource;
    T* cudaPtr;

    GlCudaBuffer(size_t size);
    ~GlCudaBuffer();


};


struct CoarseTriangle{
    std::weak_ptr<MeshPatch> patches[3];
    std::weak_ptr<DoubleStitch> doubleStitches[3];
    std::weak_ptr<TripleStitch> tripleStitches[3];
    bool isValid();
    CoarseTriangle(std::shared_ptr<MeshPatch> p1,std::shared_ptr<MeshPatch> p2,std::shared_ptr<MeshPatch> p3);
    ~CoarseTriangle();

    bool isConnectingSame3Patches(std::shared_ptr<MeshPatch> p1,std::shared_ptr<MeshPatch> p2,std::shared_ptr<MeshPatch> p3);

    bool flipToFacePos(Eigen::Vector3f pos);
};

//class MeshPatch::CoarseTriangle;


/**
 * @brief The LowDetailPoint class
 */
//template <class T>
class LowDetailPoint{
private:
public:


    int indexWithinCoarse=-1;

    bool isNeighbourWith(std::shared_ptr<LowDetailPoint> point);


    std::mutex coarseTriangleMutex;
    std::vector<std::shared_ptr<CoarseTriangle>> triangleWithinNeighbours;

    void addCoarseTriangle(std::shared_ptr<CoarseTriangle> coarseTriangle);

    std::shared_ptr<CoarseTriangle> getCoarseTriangleWith(std::shared_ptr<MeshPatch> p1,
                                                          std::shared_ptr<MeshPatch> p2,
                                                          std::shared_ptr<MeshPatch> p3);

    void cleanupCoarseTriangles();




    Eigen::Vector4f averageColor;


};



/**
 * @brief The LowDetailRenderer class
 * This class should encapsule the rendering of patches which are not loaded onto the gpu
 * each of the patch will only be represented by only one vertex within a mesh
 * Hopefully this will not introduce too many mutex locks so that everything becomest choppy and stuff.
 */
class LowDetailRenderer{
private:

    static std::weak_ptr<gfx::GLSLProgram> s_shader;
    std::shared_ptr<gfx::GLSLProgram> shader;
    static std::weak_ptr<gfx::GLSLProgram> s_geometryShader;
    std::shared_ptr<gfx::GLSLProgram> geometryShader;

    std::shared_ptr<gfx::GLSLProgram> debugShader;







    std::mutex replacingBuffers;

    GLuint VAO=0;


    std::mutex modifyingBuffers;
    std::shared_ptr<GlCudaBuffer<int>> indexBuffer;
    std::shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vertexBuffer;
    std::shared_ptr<GlCudaBuffer<int>> visibilityBuffer;
    int nrIndices;



    bool newBuffers=false;

    std::vector<int> invisibleInLastFrame;


public:
    LowDetailRenderer();
    ~LowDetailRenderer();

    void initInGlContext();


    std::vector<std::weak_ptr<MeshPatch>> patches;
    std::vector<std::weak_ptr<CoarseTriangle>> coarseTriangles;

    void addPatches(std::vector<std::shared_ptr<MeshPatch>> &patchesIn,Eigen::Vector3f camPos);



    void updateColorForPatches(std::vector<std::shared_ptr<MeshPatch>> &patchesIn);




    //TODO: split this up:
    void renderExceptForActiveSets(std::vector<std::shared_ptr<ActiveSet> > &sets, Eigen::Matrix4f proj, Eigen::Matrix4f _camPose);

    //into the following:
    void updateMaskForActiveSets(std::vector<std::shared_ptr<ActiveSet> > &sets);

    //render the color data
    void renderColor( Eigen::Matrix4f proj, Eigen::Matrix4f _camPose);

    //render the geometry for when we click onto the surface:
    void renderGeometry(Eigen::Matrix4f proj, Eigen::Matrix4f _camPose);

    void updateAllPatches();




    void downloadCurrentGeometry(std::vector<GpuCoarseVertex> &vertices,std::vector<int> &indices);

   // void updateAvgColorForPatches(std::vector<std::shared_ptr<MeshPatch>> &patches);

   // bool submit();
};

#endif
