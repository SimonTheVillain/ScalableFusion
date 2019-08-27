//
// Created by simon on 3/28/19.
//

// Example program
#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Eigen>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include <GL/glew.h>
#include <GLFW/glfw3.h>


#include <camera.h>
#include "../utils/arcball.h"
#include "../rendering/renderableModel.h"

#include "../graph/DeformationGraph.h"


#define MAX_LABELS 21



using namespace std;
using namespace Eigen;



static Arcball arcball;


//TODO: get coordinates
static float distFromBall=3.0f;
static Vector3f trans;

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

bool nextStep=false;
void key_callback(GLFWwindow* window,int key, int scancode, int action, int mods){
    if(key == GLFW_KEY_SPACE && action == GLFW_PRESS){
        nextStep = true;
    }
}



struct Intrinsics{
    int width,height;
    float cx,cy;
    float fx,fy;
    float k1,k2,k3;
    float p1,p2;

    void read(string line){
        sscanf(line.c_str(),"%d , %d , %f , %f , %f , %f , %f , %f ,%f ,%f ,%f",
                &width,&height,&cx,&cy,&fx,&fy,&k1,&k2,&k3,&p1,&p2);
    }
};

ostream & operator << (ostream &out, const Intrinsics &i)
{
    out << "cx " << i.cx << " cy " << i.cy << " fx " << i.fx << " fy " << i.fy ;
    out << " k1,k2,k3 " << i.k1 << ", " << i.k2 << ", " << i.k3 ;
    out << " p1,p2 " << i.p1 << ", " << i.p2;
    return out;
}



int main()
{
    std::string path =
            "/home/simon/datasets/structure_core/single_shots";

    std::string extrinsicsFile =
            path + "/extrinsics.txt";

    std::string intrinsicsFile =
            path + "/intrinsics.txt";



    Eigen::Matrix4f imuFromDepth;
    Eigen::Matrix4f imuFromColor;
    fstream in;
    in.open(extrinsicsFile);
    std::string line;
    std::getline(in,line);
    for(int i=0;i<16;i++){

        in >> imuFromDepth(i);
    }
    cout << imuFromDepth << endl;


    std::getline(in,line);
    std::getline(in,line);
    for(int i=0;i<16;i++){

        in >> imuFromColor(i);
    }
    cout << imuFromColor << endl;
    in.close();


    //INTRINSICS:

    Intrinsics intrinsicsColor;
    Intrinsics intrinsicsDepth;
    Intrinsics intrinsicsIr;

    in.open(intrinsicsFile);
    std::getline(in,line);
    intrinsicsColor.read(line);

    std::getline(in,line);
    intrinsicsIr.read(line);

    std::getline(in,line);
    intrinsicsDepth.read(line);

    cout << intrinsicsIr << endl;
    cout << intrinsicsColor << endl;
    cout << intrinsicsDepth << endl;



    int imageNr = 1;

    cv::Mat depth;
    cv::Mat ir;
    cv::Mat rgb;
    string imPath =
            path + "/depth/" + std::to_string(imageNr) + ".png";
    depth = cv::imread(imPath);


    imPath =
            path + "/rgb/" + std::to_string(imageNr) + ".png";
    rgb = cv::imread(imPath);


    imPath =
            path + "/ir/" + std::to_string(imageNr) + ".png";
    ir = cv::imread(imPath);

    cv::imshow("rgb",rgb);
    cv::imshow("ir",ir);
    cv::imshow("depth",depth * 10);
    cv::waitKey();
    //Eigen::Matrix4f colorToDepth =
    //std::vector<RenderVerySimpleModel::Vertex> vertices;
    for(size_t i=0;i<depth.rows;i++){
        for(size_t j=0;j<depth.cols;j++){
            //TODO: all the projection bullshit, transformation and what not
        }
    }

    //TODO: use the simple
    //RenderVerySimpleModel simpleModel;
    //simpleModel.primitiveType = GL_POINTS;
    //simpleModel.setMesh();
    return 0;

    if (!glfwInit())
        exit(EXIT_FAILURE);


    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    GLFWwindow* window;
    window = glfwCreateWindow(1280,1024,"graph", nullptr,nullptr);


    //set the callback functions for the different inputs
    glfwSetScrollCallback(window,scroll_callback);
    glfwSetCursorPosCallback(window,cursor_position_callback);
    glfwSetMouseButtonCallback(window,mouse_button_callback);
    glfwSetKeyCallback(window,key_callback);
//    glfwSetKeyCallback(window,key_callback);


    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;
    glewInit();
    glGetError();





    while(!glfwWindowShouldClose(window)){

        glClearColor(0.3f,0.3f,0.3f,1.0f);
        //glClearColor(1.0f,1.0f,1.0f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        //render:

        Matrix4f proj=Camera::projection(static_cast<float>(M_PI)*0.3f,
                                         800.0f/600.0f);
        Matrix4f translation=Matrix4f::Identity();
        translation.block<3,1>(0,3)=trans;
        //cout << "trans " << trans << endl;
        //cout << "view " << arcball.getView() << endl;
        Matrix4f transFromArcball = Matrix4f::Identity();
        transFromArcball(2,3)=-distFromBall;
        Eigen::Matrix4f mat = proj * transFromArcball * arcball.getView()*translation;
        //mat= Matrix4f::Identity();
        //mat(0,0) = mat(1,1) = mat(2,2)
        glfwSwapBuffers(window);
        glfwPollEvents();

        int width, height;
        glfwGetWindowSize(window, &width, &height);
        arcball.setFramebufferData(width,height);


        if(nextStep){
            nextStep = false;

        }




    }


    glfwDestroyWindow(window);
    glfwTerminate();




    return 0;
}
