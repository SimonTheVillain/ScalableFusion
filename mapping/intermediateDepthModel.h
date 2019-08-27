#ifndef FILE_INTERMEDIATE_DEPTH_MODEL_H
#define FILE_INTERMEDIATE_DEPTH_MODEL_H

//#include <iostream>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <GL/glew.h>
#include <shader.h>



/*
 * TODO: create an intermediate model of stuff that isn't put into the reconstruction
 * already. this should be used for rendering. and mainly camera tracking
 */



class IntermediateMap{
private:

    /* ideally this should be done in opengl... but we are lazy and therefore are doing it in
     * cuda
    gfx::GLSLProgram program;

    GLuint vertexBuffer;
    GLuint indexBuffer;
    GLuint VAO;


    GLuint FBO;
    GLuint depthRenderTex;
    GLuint depthBuf;
    */
    int width;
    int height;

    //both buffers reside on gpu
    uint16_t *inDepth;
    uint32_t *outDepth32;
    uint16_t *outDepth16;

    Eigen::Vector4f dIntrinsics;

    Eigen::Matrix4f camPoseAtCapture;

public:
    IntermediateMap(int width,int height,Eigen::Vector4f dIntrinsics);
    ~IntermediateMap();


    //void initializeInContext(int width,int height);

    //maybe also add color
    void setDepthMap(cv::Mat depth16,Eigen::Matrix4f poseAtCapture);

    cv::Mat renderDepthMap(Eigen::Matrix4f pose);

};

#endif
