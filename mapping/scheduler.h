#ifndef FILE_SCHEDULER_H
#define FILE_SCHEDULER_H


#include <thread>
#include <condition_variable>
#include <functional>
#include <queue>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include "utils/perfMeter.h"

class MeshReconstruction;
class Stream;
struct GLFWwindow;
class ActiveSet;

class Worker;

class IncrementalSegmentation;

class Segmentation;

class GarbageCollector;
namespace  gfx{
    class GpuTex2D;
}

//TODO: create a base class for the scheduler
class SchedulerBase{

public:
    static GLFWwindow* createConnectedGlContext(GLFWwindow* context);
    static void initializeGlContextInThread(GLFWwindow* context);


    virtual void pause(bool pause) = 0;
    virtual void nextStep() = 0;

    virtual Eigen::Matrix4f getLastKnownDepthPose() = 0;

    SchedulerBase(){}
    virtual ~SchedulerBase(){}
};




class SchedulerLinear : public SchedulerBase{
private:
    std::thread captureThread;

    int expandInterval = 30;//10;

    Eigen::Matrix4f lastKnownDepthPose;

    bool endThreads = false;
    GarbageCollector *garbageCollector;


    void captureWorker(std::shared_ptr<MeshReconstruction> map,Stream* stream,GLFWwindow* context);

    std::shared_ptr<IncrementalSegmentation> incrementalSegmentation;

    bool paused = false;
    bool takeNextStep = false;
public:
    SchedulerLinear(std::shared_ptr<MeshReconstruction> map,GarbageCollector *garbageCollector,
            Stream* capture,
            GLFWwindow* context,
            std::shared_ptr<IncrementalSegmentation> incrementalSegmentation);

    ~SchedulerLinear();



    //TODO: implement pause and step trough functionality
    void pause(bool pause){
        paused = pause;
    }
    void nextStep(){
        takeNextStep = true;
    }


    Eigen::Matrix4f getLastKnownDepthPose(){
        return lastKnownDepthPose;
    }

};




#endif
