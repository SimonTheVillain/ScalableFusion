//
// Created by simon on 11/14/18.
//

#include "schedulerThreaded.h"
#include "meshReconstruction.h"
#include "intermediateDepthModel.h"

#include "../datasetLoader/DatasetLoader.h"
#include "../icpCUDA/ICPOdometry.h"
#include "../gpu/ActiveSet.h"
#include "cuda/xtionCameraModel.h"
#include "GarbageCollector.h"


#include <chrono>
#include <thread>
#include <unistd.h>


//mainly for naming threads
#include <pthread.h>
#include "worker.h"



using namespace std;
using namespace Eigen;
using namespace cv;


void SchedulerThreaded::captureWorker(shared_ptr<MeshReconstruction> map, Stream *stream, GLFWwindow *context) {

    //stuff that formerly has been in capture proc:
    /**********************************/
    GLFWwindow* newContext = createConnectedGlContext(context);
    glfwMakeContextCurrent(newContext);

    //this has to be done once for every gl context or thread:
    glewExperimental = GL_TRUE;
    glewInit();
    glGetError();
    /*********************************/

    std::ofstream logMemory("/home/simon/Desktop/scalableLog.txt");

    float depthCutoff=4.0f;
    int threads=256;
    int blocks=16;


    map->initInGlLogicContext();

    Eigen::Vector4f fc=stream->getDepthIntrinsics();

    //Camera tracking via cuda icp
    ICPOdometry* odometry=new ICPOdometry(640,480,
                                          fc[2],fc[3],fc[0],fc[1]);

    IntermediateMap intermediateMap(640,480,fc);

    map->setDepthIntrinsics(stream->getDepthIntrinsics());
    map->setRGBIntrinsics(stream->getRgbIntrinsics());

    //this is a replacement for a proper test if there already is something added to the map
    bool firstLap=true;

    Sophus::SE3d accuPose;

    accuPose.setRotationMatrix(Matrix3d::Identity());
    accuPose.translation() = Vector3d(0,0,0);

    Matrix4f relDepthToColor = stream->getDepth2RgbRegistration();


    //as soon as we have a frame we create
    while(stream->isRunning() && !endThreads){

        stream->readNewSetOfImages();
        if(!stream->isRunning()){
            break; // end this loop if there is no new image
            //This could be written more efficiently (but who cares about beautiful code?)
        }
        cout << "[SchedulerThreaded::captureWorker] pulling another frame from the dataset/stream" << endl;

        Mat depth = stream->getDepthFrame(); // 16 bit 1mm resolution
        Mat rgb = stream->getRGBFrame(); // 8 bit 3 channels (usually)
        Matrix4f pose=stream->getDepthPose();

        if(depth.type() != CV_16UC1){
            assert(0);
        }

        cv::Mat depthu16;
        depth.convertTo(depthu16,CV_16UC1);//mm resolution needed needed vor ICPCUDA

        odometry->initICP((unsigned short*)depthu16.data,depthCutoff);



        if(firstLap){

            odometry->initICPModel((unsigned short*)depthu16.data,depthCutoff);
            intermediateMap.setDepthMap(depthu16,accuPose.cast<float>().matrix());

        }

        map->activeSetUpdateMutex.lock();
        shared_ptr<ActiveSet> activeSetCapturing =
                map->activeSetUpdate;
        map->activeSetUpdateMutex.unlock();

        bool hasGeometry=false;
        if(map->hasGeometry() && activeSetCapturing != nullptr){
            if(activeSetCapturing->retainedMeshPatches.size()!=0){
                hasGeometry=true;
            }
        }
        if(hasGeometry){
            //update the odometry when we already have geometry mapped
            cv::Mat reprojectedDepth =
                    map->generateDepthFromView(640,480,accuPose.cast<float>().matrix());

            odometry->initICPModel((unsigned short*)reprojectedDepth.data,depthCutoff);



            //previous: apply as long as there is no geometry
            Sophus::SE3d relPose;
            odometry->getIncrementalTransformation(relPose,threads,blocks);
            accuPose=accuPose*relPose;



        }else{
            //if there is no geometry in the reconstruction yet, we perform tracking relative to the reference frame

            Sophus::SE3d relPose;
            odometry->getIncrementalTransformation(relPose,threads,blocks);

            Mat rerender =
                    intermediateMap.renderDepthMap(accuPose.cast<float>().matrix());
            odometry->initICPModel((unsigned short*)rerender.data,depthCutoff);



        }
        odometryTimer.click();

        Matrix4f depthPose = accuPose.cast<float>().matrix();
        Matrix4f rgbPose = relDepthToColor*depthPose;


        if(stream->hasGroundTruth()){
            depthPose = stream->getDepthPose();
            rgbPose = stream->getRgbPose();
        }
        lastKnownDepthPose = depthPose;

        //upload the data to the gpu
        cv::Mat rgba;
        cv::cvtColor(rgb,rgba,CV_BGR2RGBA);

        std::shared_ptr<gfx::GpuTex2D> rgbTexture =
                std::make_shared<gfx::GpuTex2D>(
                        garbageCollector,
                        GL_RGBA,GL_RGBA,
                        GL_UNSIGNED_BYTE,
                        rgba.cols,rgba.rows,
                        true,rgba.data);



        /************************************************/
        //Prepare the sensor data

        //do the same for depth!
        Mat depthf;
        //the depth is in mm
        depth.convertTo(depthf,CV_32FC1,1.0f/1000.0f);//,1.0/5000.0);
        //create the depth map and also the depth standardDeviation on the gpu. Further elements will use this
        std::shared_ptr<gfx::GpuTex2D> depthTex =
                std::make_shared<gfx::GpuTex2D>(
                        garbageCollector,
                        GL_R32F,GL_RED,GL_FLOAT,
                        depth.cols,depth.rows,
                        true,
                        static_cast<void*>(depthf.data));
        //upload the data to the depth tex.
        std::shared_ptr<gfx::GpuTex2D> dStdTex =
                std::make_shared<gfx::GpuTex2D>(
                        garbageCollector,
                        GL_RGBA32F,GL_RGBA,GL_FLOAT,
                        depth.cols,depth.rows,
                        true,nullptr);

        //create the standard deviation Information:
        generateXtionConfidenceImage(depthTex->getCudaSurfaceObject(),
                                     dStdTex->getCudaSurfaceObject(),
                                     depth.cols,depth.rows);


        Mat dStdMat(depth.rows,depth.cols,CV_32FC4);
        dStdTex->downloadData(static_cast<void*>(dStdMat.data));



        /***************************************/
        //lets integrate the new data
        //get current active set:
        map->activeSetUpdateMutex.lock();
        std::shared_ptr<ActiveSet> activeSet =
                map->activeSetUpdate;
        map->activeSetUpdateMutex.unlock();

        //refine rgb and depth data on patches residing in the current active set
        /*
        auto refineRgb =
                std::bind(&SchedulerThreaded::refineRgb,this,
                          activeSet,rgbTexture,rgbPose);
        refineRgbWorker->setNextTask(refineRgb);
        */

        auto refineDepth =
                std::bind(&SchedulerThreaded::refineDepth,this,
                          activeSet,dStdTex,depthPose);
        refineDepthWorker->setNextTask(refineDepth);


        //update the active set as often as possible:
        //once in a while the active set is updated while the worker for the geometry integration
        //is not busy. in this case the task in the worker is starting a new process
        //also, whenever we manage the download of geometry from here
        //and the start of a new expand process.
        auto updateActiveSet =
                std::bind( &SchedulerThreaded::updateActiveSet,this,
                           dStdMat,dStdTex,depthPose,rgbTexture,rgbPose);
        updateActiveSetWorker->setNextTask(updateActiveSet);


        //debug: printing the number of used vertices into a file
        logMemory << map->m_gpuGeomStorage.vertexBuffer->getUsedElements() << endl;

        firstLap=false;

        //TODO: remove this
        //this down here is the old single threaded pipeline:
        while(!frameDoneDebugSynchronize && false){ //actially don't wait for this
            //this is not the best option
            usleep(10000000);
            //this is waaay better:
            //std::this_thread::sleep_for
        }
        frameDoneDebugSynchronize=false;


    }// end of the whole capturing process

    cout << "No new Frames: Ending capture thread!" << endl;
    delete odometry;


    //TODO: all the cleanup!!!!


    //Detroying the context which is only used in this thread
    glfwDestroyWindow(newContext);


}


void SchedulerThreaded::updateActiveSet(cv::Mat dStdMat, std::shared_ptr<gfx::GpuTex2D> dStdTex,
                                        Eigen::Matrix4f depthPose, std::shared_ptr<gfx::GpuTex2D> rgbTex,
                                        Eigen::Matrix4f rgbPose) {
    bool doExpansionUpdate=false;
    if(currentlyWorkingOnExpansion==false){
        currentlyWorkingOnExpansion=true;
        doExpansionUpdate=true;
    }



    std::shared_ptr<ActiveSet> activeSet = map->genActiveSetFromPose(depthPose);

    map->setActiveSetUpdate(activeSet);


    //DEBUG:
    /*
    std::shared_ptr<ActiveSet> activeSet = map->activeSetCapturing;
    if(activeSet==nullptr){
        activeSet = map->genActiveSetFromPose(depthPose);
        map->updateActiveSetCapturing(activeSet);
    }
    */


    //debug... check if the active set has all the geometry textures
    for(shared_ptr<MeshPatch> patch : activeSet->retainedMeshPatchesCpu){
        //TODO: test patch
        if(!patch->isPartOfActiveSetWithNeighbours(activeSet.get())){
            continue;
        }
        if(patch->geomTexPatch->gpu.lock() ==nullptr){
            cout << "[Scheduler::updateActiveSet] DEBUG/TODO: reinsert the assert at this point" << endl;

            //assert(0);//whyyyyy. the geometry textures should be secured at
            //this point
        }
    }

    updateActiveSetTimer.click();

    //if we are threaded and the last expand method is finished
    //we start a new expansion process

    if(doExpansionUpdate){ //but only if the last expansion was finished before
        //starting this expansion (otherwise we risk doubling our geometry)
        auto expand = std::bind(&SchedulerThreaded::expand,this,
                                activeSet,
                                rgbTex,rgbPose,
                                dStdTex,
                                dStdMat,depthPose);
        expandWorker->setNextTask(expand);
    }

}

void SchedulerThreaded::refineRgb(std::shared_ptr<ActiveSet> activeSet, std::shared_ptr<gfx::GpuTex2D> rgbTex,
                                  Eigen::Matrix4f rgbPose) {
    map->texturing.ColorTexUpdate(rgbTex,rgbPose,activeSet);
    refineRgbTimer.click();
}

void SchedulerThreaded::refineDepth(std::shared_ptr<ActiveSet> activeSet, std::shared_ptr<gfx::GpuTex2D> dStdTex,
                                    Eigen::Matrix4f depthPose) {
    map->geometryUpdate.Update(dStdTex,depthPose,activeSet);
    refineDepthTimer.click();
}

void SchedulerThreaded::expand(std::shared_ptr<ActiveSet> activeSet, std::shared_ptr<gfx::GpuTex2D> rgbTex,
                               Eigen::Matrix4f rgbPose, std::shared_ptr<gfx::GpuTex2D> dStdTex, cv::Mat dStdMat,
                               Eigen::Matrix4f depthPose) {
    map->geometryUpdate.Extend(activeSet,
                      dStdTex,dStdMat,depthPose,
                       rgbTex,rgbPose);
    currentlyWorkingOnExpansion = false;
    frameDoneDebugSynchronize=true;
    expandTimer.click();

}




SchedulerThreaded::SchedulerThreaded(shared_ptr<MeshReconstruction> map, Stream *stream, GLFWwindow *context) {

    lastKnownDepthPose = Eigen::Matrix4f::Identity();

    //return;//TODO: remove this desparate debug measure

    this->map = map;


    auto initializerLogic = [&](GLFWwindow* parentContext,GLFWwindow** personalContext){
        *personalContext = createConnectedGlContext(parentContext);
        initializeGlContextInThread(*personalContext);
        map->initInGlLogicContext();
    };

    auto initializer = [&](GLFWwindow* parentContext,GLFWwindow** personalContext){
        *personalContext = createConnectedGlContext(parentContext);
        initializeGlContextInThread(*personalContext);
    };


    auto cleaner = [&map](GLFWwindow** personalContext){
        map->fboStorage.forceGarbageCollect();
        map->garbageCollector->ForceCollect();
        glFinish();
        glfwDestroyWindow(*personalContext);
        delete personalContext;
    };

    //TODO: i think there was something about getting the creation and deletion of active sets into the same thread...?
    /*
    GLFWwindow* expandContext = createConnectedGlContext(context);
    expandWorker =
            new Worker(bind(initializer,expandContext),"expand");
    */
    {
        GLFWwindow **contextForWorker = new GLFWwindow*;
        expandWorker =
                new Worker(
                        bind(initializerLogic,context,contextForWorker),
                        "expand",
                        bind(cleaner,contextForWorker));
    }

    //for creating an active set and filling up the


    {
        GLFWwindow **contextForWorker = new GLFWwindow*;
        updateActiveSetWorker =
                new Worker(
                        bind(initializer,context,contextForWorker),
                        "updateSet",
                        bind(cleaner,contextForWorker));
    }
    /*
    GLFWwindow* activeSetCreationContext =
            createConnectedGlContext(context);
    updateActiveSetWorker =
            new Worker(bind(initializer,activeSetCreationContext),"updateActiveSet");
    */

    {

        GLFWwindow **contextForWorker = new GLFWwindow*;
        refineRgbWorker =
                new Worker(
                        bind(initializer,context,contextForWorker),
                        "refineRgb",
                        bind(cleaner,contextForWorker));
    }
    /*
    GLFWwindow* refineRgbContext =
            createConnectedGlContext(context);
    refineRgbWorker = new Worker(std::bind(initializer,refineRgbContext),"refineRgb");

     */

    {

        GLFWwindow **contextForWorker = new GLFWwindow*;
        refineDepthWorker =
                new Worker(
                        bind(initializer,context,contextForWorker),
                        "refineDepth",
                        bind(cleaner,contextForWorker));
    }

    /*
    GLFWwindow* refineDepthContext =
            createConnectedGlContext(context);
    refineDepthWorker = new Worker(std::bind(initializer,refineDepthContext),"refineDepth");
     */
    cout << "[Scheduler::Scheduler] TODO: delete these contexts when necessary"
         << endl;


    //if we are running non multithreaded this is the only thing thats running
    captureThread = thread(&SchedulerThreaded::captureWorker,this,map,stream,context);
    //renameThread(captureThread, "capture");
    pthread_setname_np(captureThread.native_handle(),"capture");

    odometryTimer.name = "odometry";
    updateActiveSetTimer.name = "ActiveSet";
    refineRgbTimer.name = "refineRgb";
    refineDepthTimer.name = "refineDepth";
    expandTimer.name = "expand";

#ifndef LOG_FRAMERATES
    odometryTimer.mute = true;
    updateActiveSetTimer.mute = true;
    refineRgbTimer.mute = true;
    refineDepthTimer.mute = true;
    expandTimer.mute = true;
#endif
}


SchedulerThreaded::~SchedulerThreaded() {
    //first send message to threads so they stop doing work
    endThreads = true;
    captureThread.join();
    delete expandWorker;

    delete refineRgbWorker;
    delete refineDepthWorker;
    delete updateActiveSetWorker;


}