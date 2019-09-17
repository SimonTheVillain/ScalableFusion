#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>


#include <glog/logging.h>

#include <datasetLoader.h>
#include <camera.h>
#include <memory>
#include "meshReconstruction.h"

#include <gpuTex.h>
#include "cuda/test.h"

#include <thread>
#include <map>
#include <stdexcept>

#include <boost/program_options.hpp>

#include "scheduler.h"
#include "schedulerThreaded.h"

#include "utils/arcball.h"

#include "renderableModel.h"

#include "debugRender.h"

#include "GarbageCollector.h"

#include "export/exportMap.h"
#include "../segmentation/IncrementalSegmentation.h"



//TODO: remove this
#include <cuda.h>

//how to measure memory consumption on a shell basis:
//while true; do sleep 0.1; nvidia-smi | grep mapping | grep -oh "[0-9]*MiB" >> mappingMemory.txt ; done
//while true; do sleep 0.1; nvidia-smi | grep ElasticFusion | grep -oh "[0-9]*MiB" >> elfuMemory.txt ; done
// getting klg files from pngs:
//https://github.com/HTLife/png_to_klg


//TODO: Dynamic VBO



//context sharing with cuda:
//https://blog.qt.io/blog/2015/03/03/qt-weekly-28-qt-and-cuda-on-the-jetson-tk1/


using namespace cv;
using namespace std;
using namespace Eigen;

//qt layouts without designer:
//http://www.bogotobogo.com/Qt/Qt5_LayoutNotUsingDesigner.php
static void error_callback(int error, const char* description)
{
    (void) error;
    fprintf(stderr, "Error: %s\n", description);
}

//TODO: get coordinates
static float distFromBall=3.0f;
static Vector3f trans;


static Arcball arcball;



static bool captureLeft=false;
static bool captureRight=false;
static double xposOld=0,yposOld=0;
static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    (void)window;

    float dx=static_cast<float>(xposOld-xpos);
    float dy=static_cast<float>(yposOld-ypos);
    if(captureLeft){
        float speed=0.05f;
        //trans+=Vector3f(-dx*speed,dy*speed,0);
        Vector4f trans4(-dx*speed,dy*speed,0,1);
        trans4=arcball.getView().inverse()*trans4;
        trans+=trans4.block<3,1>(0,0);
        //cout << trans << endl;
    }
    if(captureRight){
        //do the arcball thingy with the right bouse button
        arcball.drag(xpos,ypos);
    }
    xposOld=xpos;
    yposOld=ypos;
}
static bool readOutSurfaceInfo=false;
static bool centerCamera = false;
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    (void)mods;
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS){
        captureLeft=true;
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE){
        captureLeft=false;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
        captureRight=true;
        arcball.clickBegin(static_cast<float>(xposOld),
                           static_cast<float>(yposOld));
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE){
        captureRight=false;
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS &&
            glfwGetKey(window,GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS){
        readOutSurfaceInfo = true;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS &&
            glfwGetKey(window,GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS){
        centerCamera = true;
    }
}
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)window;
    (void)xoffset;
    //cout<< "dist from arcball center " << distFromBall << endl;
    float scale=0.05f;
    distFromBall*=(1.0f + static_cast<float>(yoffset)*scale);
}

static bool renderWireframe = false;
static bool renderHighDetail = true;
static int colorMode = 0;//5 would be the labelling
static int shadingMode = 0;
static bool disableRendering=false;
static bool forceDestinationGeometry = false;
static bool nextSingleStep = false;
bool paused = false;
void key_callback(GLFWwindow* window,
                  int key, int scancode, int action,
                  int mods){
    (void) window;
    (void) scancode;
    (void) mods;
    if(key == GLFW_KEY_W && action == GLFW_PRESS){
        renderWireframe = !renderWireframe;
    }
    if(key == GLFW_KEY_H && action == GLFW_PRESS){
        renderHighDetail = !renderHighDetail;
    }
    if(key == GLFW_KEY_C && action == GLFW_PRESS){
        colorMode++;
        //color mode 0 being the first texture layer
        //color mode 1 being the colorcoded patch ids
        //color mode 2 being the geometry texture
        //color mode 3 being the direct output of normals as color
        //color mode 4 being the direct output of tex references as color
        //color mode 5 shows segmentation of the first segmentation layer

        if(colorMode==6){
            colorMode=0;
        }
    }
    if(key == GLFW_KEY_S && action == GLFW_PRESS){
        shadingMode++;
        //shading mode 0 no shading
        //shading mode 1 flat shading (normals derived per triangle)
        //shading mode 2 phong shading (requires normals)
        //shading mode 3 put out the texture coordinates
        if(shadingMode==4){
            shadingMode=0;
        }
    }
    if(key == GLFW_KEY_I && action == GLFW_PRESS){
        disableRendering = !disableRendering;
    }
    if(key == GLFW_KEY_D && action == GLFW_PRESS){
        forceDestinationGeometry = !forceDestinationGeometry;
    }
    if(key == GLFW_KEY_SPACE && action == GLFW_PRESS){
        paused = !paused;
    }
    if(key == GLFW_KEY_RIGHT && action == GLFW_PRESS){
        nextSingleStep = true;
    }
}




static bool closeRequest=false;//TODO: message this to the scheduler

#include "utils/perfMeter.h"

namespace po = boost::program_options;
int main(int argc, const char * argv[])
{


    google::InitGoogleLogging(argv[0]);
    //the amount of GPU momory needed for the information header
    //cout << sizeof(GpuPatchInfo)*1024*5 * 2 << endl;
    /*
    FPSCounter count("shit");
    while(1){
        this_thread::sleep_for(chrono::microseconds(33333));
        count.click();
    }
    */

    //testBuffer();
    //return 0;

    std::string datasetPath =
            "/home/simon/datasets/tum/rgbd_dataset_freiburg3_cabinet";


    std::string graphOutputFile =
            "/home/simon/datasets/output/graph.txt";

    std::string outputPath;// =
            //"/home/simon/datasets/tum/output/";


    std::string coarseOutputFile;// =
            //"/home/simon/datasets/tum/output/coarse.ply";


    std::string detailedOutputFile;// =
            //"/home/simon/datasets/tum/output/fine.ply";
    //std::string datasetPath = "/home/simon/datasets/tumalike/track0";

    float replaySpeed=0.1f;
#ifdef DEBUG
    replaySpeed = 0.1f;
#else
    replaySpeed = 1.0f;
#endif
    bool useDatasetTrajectory = false;
    float groundtruthTrajectoryScale = 1.0f;
    bool invertGroundTruthTrajectory = false;


    //seemingly i am getting memory segmentation errors
    //https://stackoverflow.com/questions/9901803/cuda-error-message-unspecified-launch-failure
    bool headless = false;
    bool autoQuit = false;
    bool multithreaded = false;//false
    bool storeResult = true;
    int skipInitialFrames = 0;
    bool HD=false;

    float depthScale = 1.0f;


    //Manage the applications input parameters
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")
            ("input,i", po::value<std::string>(&datasetPath),
                    "Set the TUM like directory to read from.")
            ("groundtruth,t", po::bool_switch(&useDatasetTrajectory),
                    "Use the groundtruth trajectory that comes with the dataset")
            ("multithreaded,m",po::bool_switch(&multithreaded),
                    "Split the capturing process up into multiple threads")
            ("startFrame",po::value<int>(&skipInitialFrames),
                    "Skipping the first frames to start at frame n")
            ("HD,h",po::bool_switch(&HD),
                    "Using HD textures");
    desc.add_options()
            ("headless,h",po::bool_switch(&headless),
                    "Hiding windows of live reconstruction.")
            ("output,o",po::value<string>(&outputPath),
                    "Storing the reconstruction in this folder. Full reconstruction with "
                    "coarse and detailled reconstruction + "
                    "textures (not implemented).")
            ("coarse", po::value<string>(&coarseOutputFile),
                    "File to store the coarse representation in. Preferrably a ply file.")
            ("detailed", po::value<string>(&detailedOutputFile),
             "File to store the detailed representation in. Preferrably a ply file.")
            ("scaleGroundtruth", po::value<float>(&groundtruthTrajectoryScale),
                    "Scale the trajectory so it might match the scale for the depthmap")
            ("invertGroundtruth", po::bool_switch(&invertGroundTruthTrajectory),
                    "You might have inverted all of your coordinates.... if so then i advise you to set this flag.")
            ("scaleDepth,s", po::value<float>(&depthScale),"Scale the input depth so we end up in mm resolution.")
            ("autoquit,q", po::bool_switch(&autoQuit),
                    "Close the window and quit everything when finished running through the dataset")
            ("singlestep",po::bool_switch(&paused),
                    "Not autorunning but requiring single steps for updating the frame")
    ;
    //skipInitialFrames =1500;//this is for finding this weird bug in track 26/27 of the high res datasets



    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    /*
    if(headless){
        autoquit = true;
    }*/
    //if the programm runns headless then we automatically let it run headless.
    autoQuit = autoQuit || headless;


    //the translation of the current camera:
    //TODO: handle this differently
    trans[2]=-4;

    //create the context that is later used by the other thread to update the geometry
    GLFWwindow* invisibleWindow;
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(EXIT_FAILURE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_VISIBLE, 0);
    invisibleWindow = glfwCreateWindow(640, 480, "HOPE U NO VISIBLE",
                                       nullptr, nullptr);
    if (!invisibleWindow)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    //glfwMakeContextCurrent(invisibleWindow);
    glewExperimental = GL_TRUE;
    glewInit();
    glGetError();//just to get rid of that one error glew introduces at initialization
    //from here on we have opengl



    GarbageCollector garbageCollector;

    //create a map object that takes 640 by 480 sized depth images
    shared_ptr<MeshReconstruction> scaleableMap = make_shared<MeshReconstruction>(invisibleWindow,
            &garbageCollector,
            multithreaded,640,480);


    TumDataset dataset(datasetPath,true,useDatasetTrajectory,true,skipInitialFrames,
            depthScale,groundtruthTrajectoryScale,invertGroundTruthTrajectory);
    dataset.skip_count = 0;
    dataset.replay_speed = replaySpeed;

    shared_ptr<IncrementalSegmentation>  incrementalSegmentation = make_shared<EdithSegmentation>();

    SchedulerBase *scheduler = nullptr;
    if(multithreaded){
        scheduler = new SchedulerThreaded(scaleableMap,
                &dataset,
                invisibleWindow);
    }else{
        scheduler = new SchedulerLinear(scaleableMap,
                &garbageCollector,
                &dataset,
                invisibleWindow,
                incrementalSegmentation);
    }
    scheduler->pause(paused);


    //create an window with the necessary opengl context
    GLFWwindow* window;
    if (!glfwInit())
        exit(EXIT_FAILURE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_VISIBLE, 1);
    if(headless){
        glfwWindowHint(GLFW_VISIBLE, 0);
    }
    window = glfwCreateWindow(1280, 800, "SUPERMAPPING", nullptr, invisibleWindow);

    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    scaleableMap->initInGLRenderingContext();



    //initialize the debug outputs
    thatOneDebugRenderingThingy = new RenderDebugInfo();

    //TODO: implement proper destructor for these and destroy before cloing opengl context
    CameraFrustrumRenderableModel camModel(Eigen::Vector4f(0,1,0,1),//color is red
                                           Eigen::Vector4f(535.4f,539.2f,
                                                           320.1f,247.6f),//this is just about right
                                           Eigen::Vector2f(640,480),
                                           0.1f,2.0f);
    WireSphereModel wireSphereModel(Eigen::Vector4f(0,1,0,1),Eigen::Vector4f(0,0,0,1),1);



    //set the callback functions for the different inputs
    glfwSetScrollCallback(window,scroll_callback);
    glfwSetCursorPosCallback(window,cursor_position_callback);
    glfwSetMouseButtonCallback(window,mouse_button_callback);
    glfwSetKeyCallback(window,key_callback);


    gfx::GLUtils::checkForOpenGLError("[main] Error creating an opengl context!");



    while (!glfwWindowShouldClose(window) &&
            (dataset.isRunning() || !autoQuit))
    {

        scheduler->pause(paused);
        if(nextSingleStep){
            nextSingleStep = false;
            scheduler->nextStep();
        }
        //generate the view matrix
        Matrix4f proj=Camera::projection(static_cast<float>(M_PI)*0.3f,
                                         800.0f/600.0f);
        Matrix4f translation=Matrix4f::Identity();
        translation.block<3,1>(0,3)=trans;
        Matrix4f transFromArcball = Matrix4f::Identity();
        transFromArcball(2,3)=-distFromBall;
        Matrix4f view=transFromArcball*arcball.getView()*translation;

        //view.block<3,1>(0,3)+=center.block<3,1>(0,0);
        //cout << view << endl;

        glClearColor(0.3f,0.3f,0.3f,1.0f);
        //glClearColor(1.0f,1.0f,1.0f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);


        //render the
        scaleableMap->m_renderPresentation.showWireframe = renderWireframe;
        scaleableMap->m_renderPresentation.colorMode = colorMode;
        scaleableMap->m_renderPresentation.shadingMode = shadingMode;
        if(!disableRendering){
            scaleableMap->m_renderPresentation.renderInWindow(view,proj,renderHighDetail,invisibleWindow);
            //scaleableMap->render(view,proj,renderHighDetail,invisibleWindow);
        }
        if(readOutSurfaceInfo==true){
            cout << "Reading out the clicked patch to get further info" << endl;
            int patchInd;
            int triangleInd;
            Eigen::Vector4f clickedPoint =
                    scaleableMap->m_informationRenderer.renderAndExtractInfo(view,proj,
                                    renderHighDetail,invisibleWindow,
                                    1280,800,
                                    static_cast<int>(xposOld),
                                    static_cast<int>(yposOld),
                                    &patchInd,&triangleInd);
            if(!isnan(clickedPoint[0])){
                shared_ptr<MeshPatch> patch = scaleableMap->getPatchById(patchInd);
                if(patch!=nullptr){
                    wireSphereModel.setPosition(patch->getPos());
                    wireSphereModel.setRadius(patch->getRadius());
                }

            }
                                    //m_informationRenderer.renderTriangleReferencesAndDepth();
            readOutSurfaceInfo=false;

        }

        if(centerCamera==true){
            cout << "Reading out the clicked patch to get further info and center the camera" << endl;
            Eigen::Vector4f center =
            scaleableMap->m_informationRenderer.renderAndExtractInfo(view,proj,
                                    renderHighDetail,invisibleWindow,
                                    1280,800,
                                    static_cast<int>(xposOld),
                                    static_cast<int>(yposOld));
            //scaleableMap->m_informationRenderer.renderAndExtractInfo()
                                    //m_informationRenderer.renderTriangleReferencesAndDepth();
            if(!isnan(center[0])){
                trans=-center.block<3,1>(0,0);
            }
            centerCamera=false;
        }


        camModel.pose=scheduler->getLastKnownDepthPose();//.getDepthPose();
        //camModel.pose = Eigen::Matrix4f::Identity();//DEBUG
        //cout << camModel.pose << endl;
        Matrix4f projView = proj*view;
        camModel.render(projView);
        //wireSphereModel.render(projView);


        //maybe deactive all the other stuff
        thatOneDebugRenderingThingy->force_dst_geom = forceDestinationGeometry;
        thatOneDebugRenderingThingy->render(proj,view);

        //display everything that got rendered
        glfwSwapBuffers(window);
        garbageCollector.collect();
        //react to any user input
        glfwPollEvents();

        //setting the camera parameters... maybe it is wise to set this only when the window size changes
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        arcball.setFramebufferData(width,height);

        //cv::waitKey(1);//DEBUG: this blocked the rendering occasionally for several seconds
    }


    if(!graphOutputFile.empty()){
        //Exporter::storeDeformationGraph(scaleableMap.get(),graphOutputFile);
        //Exporter::storeGraph(scaleableMap.get(),graphOutputFile);
    }

    if(!coarseOutputFile.empty()){
        //TODO: make the exporter more of an integral part of the scaleableMap
        //Exporter::ExportMap(map,"/home/simon/Documents/reconstruction_results/");
        //store the stupid reconstruction somewhere
        Exporter::storeCoarse(scaleableMap.get(),coarseOutputFile);
    }

    if(!detailedOutputFile.empty()){
        //store the stupid reconstruction somewhere
        Exporter::storeFine(scaleableMap.get(),detailedOutputFile);
    }



    closeRequest=true;



    cout << "[main] DEBUG everything should be deleted" << endl;
    garbageCollector.forceCollect();
    delete scheduler;

    //erase active sets and patches so really nothing should be on the gpu
    scaleableMap->erase();


    scaleableMap.reset();


    cout << "DEBUG:most resources should be freed here! look at nvidia-smi (but in fact nothing is freed)" << endl;
    //cv::waitKey();


    cout << "Debug:set brakepoint here, the next key on the opencv window should quit the app" << endl;
    //cv::waitKey();

    //DEBUG:
    //cudaDeviceReset();//signal cuda memcheck that there is something fishy
    //return 0;

    glfwDestroyWindow(window);
    glfwDestroyWindow(invisibleWindow);
    glfwTerminate();


    cout << "Debug after deletion of at least two of the opengl contexts" << endl;
    //cv::waitKey();

    //TODO: this and the include at the very top
    void* ptr;
    cudaMalloc(&ptr,10240);
    cout << "DEBUG now exiting the program but purposefully leaving a tiny leak (cuda-memcheck sanity check)" << endl;
    cudaDeviceReset();//this is necessary for a proper memory leak analysis with cuda-memcheck
    return 0;
}
