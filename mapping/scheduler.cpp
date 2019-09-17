//this needs to be up here otherwise we have some ambiguity related issues
#include "../icpCUDA/ICPOdometry.h"

#include "scheduler.h"

#include "../datasetLoader/datasetLoader.h"

#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>


#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "meshReconstruction.h"
#include "../gpu/ActiveSet.h"

#include <chrono>
#include <thread>
#include <unistd.h>


//TODO: remove since it is not needed in this class
#include "intermediateDepthModel.h"
#include "worker.h"


#include "../segmentation/IncrementalSegmentation.h"
#include "cuda/xtionCameraModel.h"
#include <pthread.h>
#include <GarbageCollector.h>


using namespace std;
using namespace Eigen;
using namespace cv;



GLFWwindow* SchedulerBase::createConnectedGlContext(GLFWwindow *context) {
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_VISIBLE, 0);
    return glfwCreateWindow(640, 480, "HOPE U NO VISIBLE", nullptr, context);
}

void SchedulerBase::initializeGlContextInThread(GLFWwindow *context) {

    glfwMakeContextCurrent(context);

    //this has to be done once for every gl context or thread:
    glewExperimental = GL_TRUE;
    glewInit();
    glGetError();
}


SchedulerLinear::SchedulerLinear(std::shared_ptr<MeshReconstruction> map,
        GarbageCollector* garbageCollector,
        Stream *stream, GLFWwindow *context,
        shared_ptr<IncrementalSegmentation> incrementalSegmentation) :
        incrementalSegmentation(incrementalSegmentation){

    lastKnownDepthPose = Eigen::Matrix4f::Identity();

    this->garbageCollector = garbageCollector;
    captureThread = thread(&SchedulerLinear::captureWorker,this,map,stream,context);
    pthread_setname_np(captureThread.native_handle(),"capture");

}


SchedulerLinear::~SchedulerLinear() {
    endThreads = true;
    captureThread.join();
    cout << "DEBUG: scheduler destroyed" <<endl;
}



void SchedulerLinear::captureWorker(shared_ptr<MeshReconstruction> map, Stream *stream, GLFWwindow *context) {
    //TODO: Check: creating a connected context might invalidate our efforts to properly destroy all of this threads resources

    GLFWwindow* connectedContext = createConnectedGlContext(context);
    glfwMakeContextCurrent(connectedContext); // connectedContext
    //this has to be done once for every gl context or thread:
    glewExperimental = GL_TRUE;
    glewInit();
    glGetError();


    float depthCutoff=4.0f;
    int threads=256;
    int blocks=16;


    int frameCount=expandInterval;


    map->initInGlLogicContext();

    incrementalSegmentation->initInThread();

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


    shared_ptr<ActiveSet> activeSetLastExpand;
    Matrix4f depthPoseLastExpand;



    while(stream->isRunning() && !endThreads) {
        if(paused && !takeNextStep){
            continue;
        }
        takeNextStep=false;
        stream->readNewSetOfImages();
        if(!stream->isRunning()){
            break; // end this loop if there is no new image
            //This could be written more efficiently (but who cares about beautiful code?)
        }

        Mat depth = stream->getDepthFrame(); // 16 bit 1mm resolution
        Mat rgb = stream->getRgbFrame(); // 8 bit 3 channels (usually)
        Matrix4f pose=stream->getDepthPose();

        if(depth.type() != CV_16UC1){
            assert(0);
        }

        cv::Mat depthu16;
        depth.convertTo(depthu16,CV_16UC1);//mm resolution needed needed vor ICPCUDA

        odometry->initICP((unsigned short*)depthu16.data,depthCutoff);


        if(firstLap){

            odometry->initICPModel((unsigned short*)depthu16.data,depthCutoff);
            //intermediateMap.setDepthMap(depthu16,accuPose.cast<float>().matrix());

        }else{
            cv::Mat reprojectedDepth =
                    map->generateDepthFromView(640,480,
                                               accuPose.cast<float>().matrix());

            odometry->initICPModel((unsigned short*)reprojectedDepth.data,depthCutoff);

            Sophus::SE3d relPose;
            odometry->getIncrementalTransformation(relPose,threads,blocks);
            accuPose=accuPose*relPose;
        }

        map->activeSetUpdateMutex.lock();
        shared_ptr<ActiveSet> activeSetCapturing =
                map->activeSetUpdate;
        map->activeSetUpdateMutex.unlock();






        Matrix4f depthPose = accuPose.cast<float>().matrix();
        Matrix4f rgbPose = relDepthToColor*depthPose;

        //If there is groundtruth trajectory loaded, we will use it!!!
        if(stream->hasGroundTruth()){
            depthPose = stream->getDepthPose();
            rgbPose = stream->getRgbPose();
        }
        lastKnownDepthPose = depthPose;

        //upload the data to the gpu
        cv::Mat rgba;
        cv::cvtColor(rgb,rgba,COLOR_BGR2RGBA);

        std::shared_ptr<gfx::GpuTex2D> rgbTexture =
                std::make_shared<gfx::GpuTex2D>(
                        garbageCollector,
                        GL_RGBA,GL_RGBA,
                        GL_UNSIGNED_BYTE,
                        rgba.cols,rgba.rows,
                        true,rgba.data);
        rgbTexture->name = "[scheduler] rgbTexture";



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
        depthTex->name = "[scheduler] depth texture";

        //upload the data to the depth tex.
        std::shared_ptr<gfx::GpuTex2D> dStdTex =
                std::make_shared<gfx::GpuTex2D>(
                        garbageCollector,
                        GL_RGBA32F,GL_RGBA,GL_FLOAT,
                        depth.cols,depth.rows,
                        true,nullptr);
        dStdTex->name = "[scheduler] dStdTex";

        //create the standard deviation Information:
        generateXtionConfidenceImage(depthTex->getCudaSurfaceObject(),
                                     dStdTex->getCudaSurfaceObject(),
                                     depth.cols,depth.rows);


        Mat dStdMat(depth.rows,depth.cols,CV_32FC4);
        dStdTex->downloadData(static_cast<void*>(dStdMat.data));


        /*********************************************************************/
        //now integrate everything new:
        //Update the active set according to the current pose:
        //map->debugCheckTriangleNeighbourConsistency(map->GetAllPatches());
        std::shared_ptr<ActiveSet> activeSet = map->genActiveSetFromPose(depthPose);
        //map->debugCheckTriangleNeighbourConsistency(map->GetAllPatches());

        //don't ask me what this is doing here!TODO: find out
        map->clearInvalidGeometry(activeSet,depth,depthPose);

        map->geometryUpdate.update(dStdTex,depthPose,activeSet);

        map->texturing.ColorTexUpdate(rgbTexture,rgbPose,activeSet);

        //there definitely is a reason to keep the active set here!
        map->setActiveSetUpdate(activeSet);


        //every now and then we add new geometry:
        if(frameCount == expandInterval || firstLap) {


            /************* SEMANTIC LABELLING ***********************/
            if (!firstLap && false) {

                //TODO: we actually always want to use the pose and activeSet from the last expand step
                //THIS IS VERY VALID!!!!!!!!!!!!!!!!!!!!!!!!!!!

                //First we need to render the labels in all possible constellations
                Mat reprojectedDepth =
                        map->generateDepthFromView(640, 480,
                                                   depthPoseLastExpand);
                imshow("reprojected depth", reprojectedDepth);


                //render all the necessary info needed for labelling
                Matrix4f proj = map->genDepthProjMat();
                Mat renderedDepth(depth.rows, depth.cols, CV_32FC1);
                Mat renderedNormals(depth.rows, depth.cols, CV_32FC4);
                Mat renderedLabels(depth.rows, depth.cols, CV_32FC4);//SC1?
                Mat renderedColor(depth.rows, depth.cols, CV_32FC4);
                map->m_informationRenderer.render(activeSet.get(),//activeSetExpand
                                                  proj, depthPoseLastExpand,
                                                  &renderedDepth, &renderedNormals, &renderedColor,
                                                  &renderedLabels);



                Mat novellabels =
                        incrementalSegmentation->generateNewLabels(&renderedDepth,
                                &renderedNormals,
                                &renderedColor,
                                &renderedLabels);

                //then we run the labelling

                imshow("renderedDepth", renderedDepth * 0.25f);
                imshow("renderedNormals", renderedNormals);
                imshow("renderedLabels", renderedLabels);
                imshow("renderedColor", renderedColor);
                cout << renderedLabels.at<cv::Vec4i>(100, 100) << endl;


                cv::waitKey();


                //now we project the labels
                //fake labels, best labes
                Mat newLabels(depth.rows, depth.cols, CV_32SC4);
                newLabels.setTo(Scalar(100000, 1, 1));
                imshow("newLabels", newLabels);
                //maybe instead of the dStdTex we use the new label texture
                map->labelling.projectLabels(activeSet, newLabels, dStdTex, depthPoseLastExpand);//activeSet


            }
            /*********************************************************/


            //expanding the existing geometry

            /*
            map->applyNewData(activeSet,
                              dStdTex,dStdMat,depthPose,
                              rgbTexture,rgbPose);
            */
            map->geometryUpdate.extend(activeSet,
                                       dStdTex, dStdMat, depthPose,
                                       rgbTexture, rgbPose);



            //setting the active set, which also gets rendered to
            //the one updated in the expand method.
            //only do this in single threaded mode.
            map->setActiveSetUpdate(map->activeSetExpand);

            frameCount = 0;

            activeSetLastExpand = map->activeSetExpand;
            depthPoseLastExpand = depthPose;
            //waitKey();//DEBUG
        }

        frameCount++;

        //debug: printing the number of used vertices into a file

        firstLap=false;


        //cleanup VBO and VAOs that are deleted but only used within this thread
        map->cleanupGlStoragesThisThread();
        garbageCollector->collect();

        cv::imshow("rgb",rgb);
        cv::waitKey(1);
        //tons of debug output to find this fucking memory leak!!!!
        int texCount =
                map->texAtlasStds->countTex() +
                map->texAtlasGeomLookup->countTex() +
                map->texAtlasRgb8Bit->countTex() +
                map->texAtlasSegLabels->countTex();
        cout << "texCount overall: " << texCount <<
        " stds " << map->texAtlasStds->countTex() << " lookup " <<
        map->texAtlasGeomLookup->countTex() << " rgb " << map->texAtlasRgb8Bit->countTex() <<   endl;

        int patchCount =
                map->texAtlasStds->countPatches() +
                map->texAtlasGeomLookup->countPatches() +
                map->texAtlasRgb8Bit->countPatches() +
                map->texAtlasSegLabels->countPatches();
        cout << "patchCount overall: " << patchCount <<
             " stds " << map->texAtlasStds->countPatches() << " lookup " <<
             map->texAtlasGeomLookup->countPatches() << " rgb " << map->texAtlasRgb8Bit->countPatches() <<   endl;


        cout << "FBOs active " << map->getFboCountDebug() << endl;

    }
    //delete everything for the fbo
    map->fboStorage.forceGarbageCollect();
    garbageCollector->forceCollect();
    glFinish();


    glfwDestroyWindow(connectedContext);
    delete odometry;

    cout << "DEBUG: scheduler finished thread" <<endl;
}