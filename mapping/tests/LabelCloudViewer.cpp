//
// Created by simon on 3/28/19.
//

// Example program
#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Eigen>

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
int main()
{
    std::string labelColorsFile =
            "/home/simon/datasets/EdithCloudBla/label_color_map.txt";

    std::string pointsFile =
            "/home/simon/datasets/EdithCloudBla/coordsConf.txt";


    struct Point{
        Vector4f pos;
        int label[MAX_LABELS];
        float confidence[MAX_LABELS];
    };
    vector<Point> points;
    vector<Eigen::Vector4f> colors;

    fstream file;
    file.open(labelColorsFile);

    if(!file.is_open()){
        assert(0);
    }


    std::string line;
    while (std::getline(file, line)){
        std::istringstream iss(line);
        int r,g,b;
        char separator;
        iss >> r >> separator >> g >> separator >> b;
        cout << r << " " << g << " " << b << endl;
        colors.push_back(Vector4f(r/255.0f,g/255.0f,b/255<.0f,1.0f));
    }
    cout << "yeah" << endl;
    file.close();
    file.open(pointsFile);
    if(!file.is_open()){
        assert(0);
    }

    while(std::getline(file,line)){

        Point p;
        std::istringstream iss(line);
        float x,y,z;
        iss >> x >> y >> z;
        cout << x << " " << y << " " << z << " ";
        p.pos = Vector4f(x,y,z,1.0f);
        for(int i=0;i<MAX_LABELS;i++){
            iss >> p.label[i] >> p.confidence[i];
            cout << p.label[i] << " " << p.confidence[i] << " ";
        }
        cout << endl;
        points.push_back(p);
    }







    int nodeCount;
    int edgeCount;
    file >> nodeCount;
    cout << nodeCount << endl;
    file >> edgeCount;

    vector<Vector4f> nodes(nodeCount);
    vector<Vector3f> nodes3(nodes.size());
    for(size_t i = 0;i<nodeCount;i++){
        Vector4f p;
        file >> p[0] >> p[1] >> p[2];
        p[3] = 1;
        //cout << p << endl;
        nodes[i] = p;
        nodes3[i] = Vector3f(p[0],p[1],p[2]);
        nodes3[i]*=0.1f;
    }
    vector<unsigned int> edges(edgeCount*2);
    for(size_t i=0;i<edgeCount;i++){
        int edge1,edge2;
        file >> edge1 >> edge2;// >> edges[i*2 + 1];
        edges[i*2] = edge1;
        edges[i*2 + 1] = edge2;
        //cout << edge1 << " " << edge2 << endl;
    }

    file.close();
    std::cout << "Graph is loaded! Hopefully!" << std::endl;



    DeformationGraph deformationGraph;

    deformationGraph.indices = edges;
    deformationGraph.nodes = nodes3;

    PinConstraint cons;
    cons.node = 0;
    cons.pos = nodes3[0];
    //cons.pos[1] +=1.0f;
    deformationGraph.pin_constraints.push_back(cons);//only one constraint should shift everything towards it


    cons.node = 1;
    cons.pos = nodes3[1];
    //cons.pos[1] +=1.5f;
    //cons.pos[0] +=0.5f;
    deformationGraph.pin_constraints.push_back(cons);//only one constraint should shift everything towards it


    cons.node = 2;
    cons.pos = nodes3[2];
    //cons.pos[1] +=1.0f;
    deformationGraph.pin_constraints.push_back(cons);//only one constraint should shift everything towards it

    cons.node = 3;
    cons.pos = nodes3[3];
    //cons.pos[1] +=1.0f;
    //cons.pos[0] +=0.5f;
    deformationGraph.pin_constraints.push_back(cons);//only one constraint should shift everything towards it

    for(int i=0;i<100;i++){
        cons.node = i;
        cons.pos = nodes3[i];
        deformationGraph.pin_constraints.push_back(cons);

    }

    for(int i=deformationGraph.nodes.size()-100;i<deformationGraph.nodes.size();i++){
        cons.node = i;
        cons.pos = nodes3[i];
        cons.pos[1] += 0.5;
        deformationGraph.pin_constraints.push_back(cons);
    }



    deformationGraph.generateNeighbourList();
    deformationGraph.initializeTransforms();
    //deformationGraph.G_rot[1](1) = 1;
    for(int i=0;i<1;i++){

        deformationGraph.generateResiduals();
        deformationGraph.generateJacobian();
        deformationGraph.gaussNewtonStep();

    }










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

    //******************SETTING UP SHADER***************************


    GLuint ssboColors;
    glGenBuffers(1, &ssboColors);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboColors);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Vector4f) * colors.size(), &colors[0], GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssboColors);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); // unbind


    GLuint ssboPoints;

    //**************************************************************




    RenderVerySimpleModel simpleModel;
    simpleModel.setMesh(nodes,edges);


    RenderVerySimpleModel simpleModel2;
    simpleModel2.color = Vector4f(1,1,0,1);
    vector<Vector4f> nodes2 = nodes;

    for(int i=0;i<nodes.size();i++){

        nodes2.push_back(nodes[i]);
        nodes2[i](0) += deformationGraph.G_t[i](0);
        nodes2[i](1) += deformationGraph.G_t[i](1);
        nodes2[i](2) += deformationGraph.G_t[i](2);
    }

    simpleModel2.setMesh(nodes2,edges);


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
        //mat(0,0) = mat(1,1) = mat(2,2) = 0.01f;

        simpleModel.render(mat);
        simpleModel2.render(mat);


        glfwSwapBuffers(window);
        glfwPollEvents();

        int width, height;
        glfwGetWindowSize(window, &width, &height);
        arcball.setFramebufferData(width,height);


        if(nextStep){
            nextStep = false;

            deformationGraph.generateResiduals();
            deformationGraph.generateJacobian();
            deformationGraph.gaussNewtonStep();

            vector<Vector4f> nodes2;
            for(int i=0;i<nodes.size();i++) {

                nodes2.push_back(nodes[i]);
                nodes2[i](0) += deformationGraph.G_t[i](0);
                nodes2[i](1) += deformationGraph.G_t[i](1);
                nodes2[i](2) += deformationGraph.G_t[i](2);
            }
            simpleModel2.setMesh(nodes2,edges);
        }



    }


    glfwDestroyWindow(window);
    glfwTerminate();




    return 0;
}
