#ifndef FILE_DATASET_LOADER_H
#define FILE_DATASET_LOADER_H

//should i include this here?
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

#include <vector>
#include <chrono>

//using namespace Eigen;
//using namespace std;
//using namespace cv;
namespace radical{
    class RadiometricResponse;
    class VignettingResponse;
}

class Stream{
public:
    virtual bool isRunning() = 0;
    virtual cv::Mat getDepthFrame() = 0;
    virtual cv::Mat getRGBFrame() = 0;
    virtual Eigen::Vector4f getDepthIntrinsics() = 0;
    virtual Eigen::Vector4f getRgbIntrinsics() = 0;
    virtual Eigen::Matrix4f getDepth2RgbRegistration() = 0;
    virtual bool hasGroundTruth() = 0;
    virtual Eigen::Matrix4f getDepthPose() = 0;
    virtual Eigen::Matrix4f getRgbPose() = 0;
    //virtual Eigen::Matrix4f getRgbToDepthTransform() = 0;
    virtual void readNewSetOfImages() = 0;
    //TODO: add pose as quaternion + translation

};


class TumDataset : public Stream{
private:
    int frameshiftRGB=0;//seemingly this is not an issue for the new setup
    //bool firstFrame=true;
    bool readDepth;
    bool readRGB;
    bool running = false;
    //std::ifstream frameList;
    std::string folderPath;
    cv::Mat currentDepth;
    cv::Mat currentRGB;

    Eigen::Vector4f depthIntrinsics;
    Eigen::Vector4f rgbIntrinsics;
    Eigen::Matrix4f depth2RGB;
    float scaleDepth;

    //these are not needed because it can be done right when loading
    /*
    float trajectoryScale = 1.0f;
    bool invertTrajectory = false;
     */


    //TODO: set this accordingly
    double currentTimestamp;

    //TODO: store the trajectory in these groundtruth files:
    struct TrajectoryPoint{
        double timestamp;
        Eigen::Matrix4f position;
    };

    std::vector<TrajectoryPoint> trajectory;

    void loadIntrinsics();

    std::chrono::system_clock::time_point lastFrameReadout;


    bool hasPoses;
    bool _hasHighRes = false;


    bool isIdsSetup=false;
    //cv::Mat rgbFundamental, depthFundamental;
    cv::Mat rgbUndistort1,rgbUndistort2, depthUndistort1,depthUndistort2;



    radical::RadiometricResponse* radiometricResponse = nullptr;
    radical::VignettingResponse* vignettingResponse = nullptr;

    int frameIndex = 0;
    std::vector<double> timestamps;
    std::vector<std::string> rgbFiles;
    std::vector<std::string> depthFiles;
    std::vector<float> exposureTimes;
    float _rgbExposureTime;
    cv::Vec3f whiteFix;


public:
    float replaySpeed=0;

    bool realtime;//it anyway is not implemented yet.
    TumDataset(std::string folder,bool realtime=false,bool usePose=false,bool useHighRes = true,int skipNFrames = 0,
            float depthScale = 1.0f,float trajectoryGTScale = 1.0f,bool invertGTTrajectory = false);



    virtual ~TumDataset();

    bool isRunning();

    void readNewSetOfImages();
    cv::Mat getDepthFrame();
    cv::Mat getRGBFrame();
    float getRGBExposure();
    bool hasHighRes();
    Eigen::Vector4f getDepthIntrinsics();
    Eigen::Vector4f getRgbIntrinsics();
    Eigen::Matrix4f getDepth2RgbRegistration();
    bool hasGroundTruth();
    Eigen::Matrix4f getDepthPose();
    Eigen::Matrix4f getRgbPose();



    int skipCount=20;

};
/*
class GeorgDataset : public Stream{
//todo all of this
};
*/

#endif
