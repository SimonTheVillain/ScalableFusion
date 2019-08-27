#ifndef FILE_CAMERA_H
#define FILE_CAMERA_H
#include <eigen3/Eigen/Eigen>
#include <opencv2/core.hpp>

//using namespace Eigen;

class Camera{
public:
    static Eigen::Matrix4f lookFromAt(Eigen::Vector4f eye, Eigen::Vector4f center, Eigen::Vector4f up);

    static Eigen::Matrix4f projection(float fovY, float aspectRatio, float zFar=100.0f, float zNear=0.01f);

    /**
     * @brief calcCamPosFromExtrinsic
     * Todo: this doesn't really fit in here... (but maybe it does) i don't know actually
     * @param cam
     * @return
     */
    static Eigen::Vector4f calcCamPosFromExtrinsic(Eigen::Matrix4f cam);

    static Eigen::Matrix4f genProjMatrix(Eigen::Vector4f intrinsics);


    static Eigen::Matrix4f genScaledProjMatrix(Eigen::Vector4f intrinsics,
                                               cv::Size2i res);

};


#endif
