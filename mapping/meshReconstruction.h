#ifndef FILE_MESH_RECONSTRUCTION_H
#define FILE_MESH_RECONSTRUCTION_H

#include "base/meshStructure.h"
#include "mapInformationRenderer.h"
#include "mapPresentationRenderer.h"
#include "lowDetailMapRenderer.h"

//TODO: migrate functionality to these new classes
#include "Texturing.h"
#include "GeometryUpdate.h"
#include "Meshing.h"
#include "Stitching.h"
#include "Labelling.h"

#include <eigen3/Eigen/Eigen>

#include <opencv2/core.hpp>

#include <map>

//synchronization
#include <condition_variable>
#include <thread>
#include <chrono>
#include <atomic>


#include "gpu/gpuGeomStorage.h"
#include "utils/octree.h"
#include "gpu/threadSafeFBO_VAO.h"


class PreSegmentation;
class GpuNormSeg;
class TexAtlas;

namespace gfx {
class GLSLProgram;
class GpuTex2D;
} // namespace gfx

class Worker;

class SchedulerLinear;// : public Scheduler;
class SchedulerThreaded;// : public Scheduler;
struct Edge;

class MapInformationRenderer;
class MapPresentationRenderer;
class LowDetailRenderer;

class GarbageCollector;
class Exporter;


/**
 *Thoughts about spanning work over threads:
 * https://stackoverflow.com/questions/21010932/c11-parallel-for-implementation
 */


class MeshReconstruction{
    friend MapInformationRenderer;
    friend MapPresentationRenderer;
    friend LowDetailRenderer;
    friend Scheduler; //maybe we don't want this one as friend
    friend SchedulerLinear;
    friend SchedulerThreaded;
    friend ActiveSet;
    friend MeshTexture;
    friend Stitching;
    friend GeometryUpdate;
    friend Texturing;
    friend Labelling;
    friend Exporter;
    friend Meshing;
    //also we might not want
public:

    //TODO: use these to replace everything else
    struct Parameters{
        //size_t maxLabelCount = 8;
        size_t maxSemLabelCount = 8;
        cv::Size2i depthRes;
        cv::Size2i rgbRes;
        // These variables decides if and when new textures should be added or removed
        int maxNrSimilarTextures = 3;
        float maxDepthFactorThreshForTexAdding = 0.6f;
        float maxDepthFactorThreshForTexReplacement = 0.3f;

        float maxAngleFactorThreshForTexAdding = 0.8f;
        float maxAngleFactorThreshForTexReplacement = 0.5f;

        float maxDepthStep = 0.1f;
        float maxDistance = 3.0f;//distance cutoff

        Eigen::Vector4f rgbfxycxy;
        //Eigen::Vector2i rgbRes;
        Eigen::Vector4f depthfxycxy;
        //Eigen::Vector2i depthRes;
        Eigen::Matrix4f camPosition;//maybe this should not be in here!!!

    };
    Parameters params;



    bool removePatch(std::shared_ptr<MeshPatch> patch);
    std::shared_ptr<MeshPatch> getPatchById(int id){
        m_patchesMutex.lock();
        std::shared_ptr<MeshPatch> patch = m_patches[id];
        m_patchesMutex.unlock();
        return patch;
    }

    void clearInvalidGeometry(std::shared_ptr<ActiveSet> set,cv::Mat depth, Eigen::Matrix4f depthPose);

private:
    GarbageCollector* garbageCollector;

    //shouldnt that be a shared pointer
    std::mutex m_patchesMutex;
    std::map<int,std::shared_ptr<MeshPatch>> m_patches;
    Octree<MeshPatch> octree;//stores the objects in a spacial manner
    int m_currentMaxPatchID=0;//this basically is the number of patches currently in use



    //TODO: completely get rid of the concept of recycler
    ThreadSafeFBOStorage fboStorage;
    //ThreadSafeVAOStorage vaoStorage;

    void cleanupGlStoragesThisThread();

    /**
     * @brief preSeg
     * This is supposed to be an interface for interchangeable pre segmentation
     */
    std::shared_ptr<PreSegmentation> preSeg;
    std::shared_ptr<GpuNormSeg> gpuPreSeg;




    /****************THIS IS A VALID COMMENT THINK ABOUT IT******************/
    //
    //these are 4 channel 32bit floats right now
    //1: really this is an 32bit integer with an absolute triangle index
    //2,3,4: the barycentric coordinates on the triangle
    //TODO: reduce these to 4 channel 16 bit float
    //1,2: combined they are one 32Bit integer
    //3,4: 2 parts of the barycentric coordinate.
    //the third one is obtained by 1 - byrycentric1 -barycentric2
    std::shared_ptr<TexAtlas> texAtlasGeomLookup;

    //Standard deviations of the surfaces
    std::shared_ptr<TexAtlas> texAtlasStds;//[2];

    //at the moment we only store the SDR versions of the textures
    std::shared_ptr<TexAtlas> texAtlasRgb8Bit;


    //TODO check which is best for our task
    //pure 32 bit integers
    std::shared_ptr<TexAtlas> texAtlasSegLabels;
    //int32 x 4 but interpreted as pairs of 16bit integers and
    //16 bit (float) weights
    std::shared_ptr<TexAtlas> texAtlasSemSegLabelsWeights;




    GpuGeomStorage m_gpuGeomStorage;
public: //TODO: remove, just for debugging purpose
    MapInformationRenderer m_informationRenderer;
    MapPresentationRenderer m_renderPresentation;
    LowDetailRenderer lowDetailRenderer;

    float getMaxDistance(){return params.maxDistance;}
private:

    int getFboCountDebug(){
        return fboStorage.getTotalFboCount();
    }

    //condition variable used to synchronize generation of the gpu buffer:
    //http://en.cppreference.com/w/cpp/thread/condition_variable/wait
    bool m_glLogicInitialized=false;
    std::condition_variable m_conditionVariable;
    std::mutex m_conditionVariableMutex;

   cv::Mat generateColorCodedTexture(cv::Mat segmentation);

   void setActiveSetUpdate(std::shared_ptr<ActiveSet> set);



    TriangleReference addTriangle(VertexReference pr1, VertexReference pr2, VertexReference pr3,
            std::vector<std::weak_ptr<GeometryBase>> &debugNewStitches);
    TriangleReference addTriangle(VertexReference pr1, VertexReference pr2, VertexReference pr3);


//USEFUL FUNCTIONS TO CHECK FOR CONSISTENCY
    void debugCheckTriangleNeighbourConsistency(std::vector<std::shared_ptr<MeshPatch>> patches);
    void debugCheckTriangleEdgesUnregistered(std::vector<std::shared_ptr<MeshPatch>> patches);

    /*//TODO: Get rid of this and put it to the Meshing class
    void meshIt(cv::Mat points, cv::Mat meshPointers, cv::Mat vertexIndices, cv::Mat sensorStd,float maxDepthStep, Eigen::Matrix4f depthPose, Eigen::Matrix4f rgbPose);
    */


    Worker *renderingActiveSetUpdateWorker=nullptr;
    std::mutex activeSetRenderingMutex;
    std::shared_ptr<ActiveSet> activeSetRendering;


public:
    GeometryUpdate geometryUpdate;
    Texturing texturing;
    Labelling labelling;
    //Meshing meshing;


    std::mutex activeSetUpdateMutex;
    std::shared_ptr<ActiveSet> activeSetUpdate;

    std::shared_ptr<ActiveSet> activeSetExpand;



    MeshReconstruction(GLFWwindow *context, GarbageCollector *garbageCollector,bool threaded,
                 int depthWidth=640, int depthHeight=480,
                 int rgbWidth=640, int rgbHeight=480);
    ~MeshReconstruction();

    //TODO: these two
    std::shared_ptr<MeshPatch> genMeshPatch();


    std::shared_ptr<MeshTexture> genMeshTexture(
            MeshTexture::Type contentType);

    void setRGBIntrinsics(Eigen::Vector4f fxycxy);
    void setDepthIntrinsics(Eigen::Vector4f fxycxy);


    cv::Mat generateDepthFromView(int width, int height, Eigen::Matrix4f pose);



    Eigen::Matrix4f genDepthProjMat();


    /**
     * @brief initInGlLogicContext
     */
    void initInGlLogicContext();
    std::atomic<bool> initializingLogic;

    /**
     * @brief initInGLRenderingContext
     */
    void initInGLRenderingContext();



    bool hasGeometry();






    //pls describe these
    std::shared_ptr<ActiveSet> genActiveSetFromPose(Eigen::Matrix4f depthPose);



    std::vector<cv::Rect2f> genBoundsFromPatches(std::vector<std::shared_ptr<MeshPatch> > &patches,
                                                 Eigen::Matrix4f pose, Eigen::Matrix4f proj,
                                                 std::shared_ptr<ActiveSet> activeSet);





    //Free everything
    void erase();



    std::vector<std::shared_ptr<MeshPatch>> GetAllPatches();


};


#endif
