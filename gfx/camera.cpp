#include "camera.h"
#include <math.h>


#include <iostream>
using namespace std;

using namespace Eigen;
//http://stackoverflow.com/questions/349050/calculating-a-lookat-matrix
//but i really do not want to think for myself
Matrix4f Camera::lookFromAt(Vector4f eye, Vector4f at,Vector4f up)
{
    assert(0);//this is untested but might work


    Matrix4f lookAt = Matrix4f::Identity();

    Vector3f direction = (at-eye).block<3,1>(0,0).normalized();
    //angle = Vector3f(0,0,-1);

    //Eigen::Vector3f axis;

    Quaternionf q1 = Quaternionf::FromTwoVectors(Vector3f(0,0,-1),direction);//AngleAxisf(angle,axis);
    Vector3f intermediateUp = q1 * Vector3f(0,1,0);
    Vector3f upOnImagePlane = up.block<3,1>(0,0) - direction * up.block<3,1>(0,0).dot(direction);
    Quaternionf q2 = Quaternionf::FromTwoVectors(intermediateUp,upOnImagePlane);
    Quaternionf q = q2 * q1;
    Matrix3f rot = q.normalized().toRotationMatrix();

    lookAt.block<3,3>(0,0) = rot;
    lookAt.block<3,1>(0,3) = eye.block<3,1>(0,0);

    //Matrix4f rot1 = Matrix4f::rot
    return lookAt;
}

Matrix4f Camera::projection(float fovY, float aspectRatio,float zFar,float zNear)
{
    //http://www.songho.ca/opengl/gl_projectionmatrix.html
    float yScale = cos(fovY*0.5f)/sin(fovY*0.5f);

    float xScale = yScale/aspectRatio;
    Matrix4f proj;
    proj <<       xScale, 0, 0, 0,
                0, yScale, 0, 0,
                0, 0, -(zFar+zNear)/(zFar-zNear), -2*zNear*zFar/(zFar-zNear),
                0, 0, -1, 0;
    return proj;
}

Eigen::Vector4f Camera::calcCamPosFromExtrinsic(Eigen::Matrix4f cam)
{
    //cout << "[Camera::calcCamPosFromExtrinsic] you could also multiply (0,0,0,1) with the cam matrix.(tested and confirmed)" << endl;
    Matrix3f R = cam.block<3,3>(0,0);
    Vector3f t = cam.block<3,1>(0,3);
    Vector3f pos = -R.transpose()*t;
    return Vector4f(pos[0],pos[1],pos[2],1);
}

Matrix4f Camera::genProjMatrix(Vector4f intrinsics)
{
    float fx=intrinsics[0];
    float fy=intrinsics[1];
    float cx=intrinsics[2];
    float cy=intrinsics[3];
    Matrix4f proj;//one to do what has to be done anyway
    proj << fx, 0,  cx, 0,
            0,  fy, cy, 0,
            0,  0,  0,  -1,
            0,  0, 1,  0;
    return proj;
}

Matrix4f Camera::genScaledProjMatrix(Vector4f intrinsics, cv::Size2i res)
{
    float fx=intrinsics[0];
    float fy=intrinsics[1];
    float cx=intrinsics[2];
    float cy=intrinsics[3];
    Matrix4f proj;//one to do what has to be done anyway
    proj << fx, 0,  cx, 0,
            0,  fy, cy, 0,
            0,  0,  0,  -1,
            0,  0, 1,  0;


    float w=res.width;
    float h=res.height;
    float zmin=0.1f;
    float zmax=30.0f;
    float b=2.0f/(1.0f/zmin-1.0f/zmax);
    float a=b/zmax+1.0f;
    Matrix4f proj2;//the other one to scale everything to the normalized coordinates
    proj2 << 2.0f/(w),    0,  0,  -1.0f+1.0f/w,
             0,     2.0f/(h), 0,  -1.0f+1.0f/h,
            0,          0,  b,  a,
            0,          0,  0,  1;
    return proj2*proj;
}
