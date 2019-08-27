//
// Created by simon on 11/14/18.
//

#ifndef SUPERMAPPING_SCHEDULERTHREADED_H
#define SUPERMAPPING_SCHEDULERTHREADED_H
#include "scheduler.h"

class SchedulerThreaded : public SchedulerBase{
private:

    //TODO: this
    GarbageCollector* garbageCollector;
    //bool usingGroundtruth;
    bool stepTrough=true;
    //bool threaded; //This is always threaded!!!!!!!!
    bool endThreads=false;


    bool currentlyWorkingOnExpansion=false;

    int expandInterval = 10;//this means every 10 frames, the map gets expanded
    //(only when we are not in the threaded mode)
    int frameCount=expandInterval;


    std::shared_ptr<MeshReconstruction> map;

    Segmentation* attentionSegmenter;

    std::thread captureThread;
    //GLFWwindow* captureContext;


    //Worker* refineWorker;


    //TODO: these three
    Worker* updateActiveSetWorker;
    Worker* refineRgbWorker;
    Worker* refineDepthWorker;

    Worker* expandWorker;


    FPSCounter odometryTimer;
    FPSCounter updateActiveSetTimer;
    FPSCounter refineRgbTimer;
    FPSCounter refineDepthTimer;

    FPSCounter expandTimer;

    Worker* uploadWorker;//TODO: creates the active set
    // ideally the upload process is taking place while the textures
    //and the geometry is getting refined so that they can work on the patches
    //visible in the last frame, and the expansion worker can work on the new textures
    //PROBLEMATIC: Uploading textures from the cpu that got newer(better) gpu counterparts
    //at the same moment.

    //There is a separate thread for downloading stuff in the mapping
    //class itself. This might cause some fragmentation in the code:
    //TODO: unify the scheduler and the map class. (somehow)
    //maybe we even just give the ownership of the scheduler to map class.




    //TODO: these three:
    /*
     * The active set is coming from a method which just loads the visible patches
     * which stores the active set into the activeSetReconstruction variable.
     * The update methods read out the active set from exactly this capturing active set.
     * The second method storing to this active set is the expand method,
     * the update and expand method have to make sure not to overwrite their contents
     * if they do not want to.
     */
    //std::shared_ptr<ActiveSet> genActiveSet(Eigen::Matrix4f depthPose); //should be part of map
    void updateActiveSet(cv::Mat dStdMat, std::shared_ptr<gfx::GpuTex2D> dStdTex, Eigen::Matrix4f depthPose,
                         std::shared_ptr<gfx::GpuTex2D> rgbTex, Eigen::Matrix4f rgbPose);

    void refineRgb(std::shared_ptr<ActiveSet> activeSet,
                   std::shared_ptr<gfx::GpuTex2D> rgbTex,
                   Eigen::Matrix4f rgbPose);

    void refineDepth(std::shared_ptr<ActiveSet> activeSet,
                     std::shared_ptr<gfx::GpuTex2D> dStdTex, Eigen::Matrix4f depthPose);


    void expand(std::shared_ptr<ActiveSet> activeSet,
                std::shared_ptr<gfx::GpuTex2D> rgbTex, Eigen::Matrix4f rgbPose,
                std::shared_ptr<gfx::GpuTex2D> dStdTex,
                cv::Mat dStdMat, Eigen::Matrix4f depthPose);




    void segmentLabel(std::shared_ptr<ActiveSet> activeSet,
                      cv::Mat rgb,
                      cv::Mat depth,
                      std::shared_ptr<gfx::GpuTex2D> dStdTex,
                      Eigen::Matrix4f pose);


    bool frameDoneDebugSynchronize=false;


    //actually we should secure this via a mutex
    Eigen::Matrix4f lastKnownDepthPose;



    void captureWorker(std::shared_ptr<MeshReconstruction> map,Stream* stream,GLFWwindow* context);
public:

    SchedulerThreaded(std::shared_ptr<MeshReconstruction> map,Stream* stream,GLFWwindow* context);
    ~SchedulerThreaded();



    void pause(bool pause){
        assert(0); //unimplemented
    }
    void nextStep(){
        assert(0); // unimplemented
    }


    Eigen::Matrix4f getLastKnownDepthPose(){
        return lastKnownDepthPose;
    }
};



#endif //SUPERMAPPING_SCHEDULERTHREADED_H
