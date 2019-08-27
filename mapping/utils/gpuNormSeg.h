#ifndef FILE_GPU_NORM_SEG_H
#define FILE_GPU_NORM_SEG_H

#include <gpuTex.h>
#include <memory>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

class GpuNormSeg{
private:
    std::shared_ptr<gfx::GpuTex2D> dStdMaxStd;
    std::shared_ptr<gfx::GpuTex2D> points;
    std::shared_ptr<gfx::GpuTex2D> normals;
    std::shared_ptr<gfx::GpuTex2D> gpuSegmentation;
    cv::Mat existingGeometry;
    cv::Mat sensorStds;
    cv::Mat existingStds;
    cv::Mat segResult;
    int segCount=-1;
public:
    GpuNormSeg(GarbageCollector* garbageCollector,int width,int height);
     ~GpuNormSeg();


    void calcNormals();
    std::shared_ptr<gfx::GpuTex2D> getGpuNormals();


    void calcPoints(std::shared_ptr<gfx::GpuTex2D> dStdMaxStd,Eigen::Vector4f fxycxy);
    std::shared_ptr<gfx::GpuTex2D> getGpuPoints();

    void segment();
    std::shared_ptr<gfx::GpuTex2D> getGpuSegmentation();
    cv::Mat getSegmentation();
    int getSegCount();




    int maxNrPointsPerSegment = 800;
    int maxExtentPerSegment = 30;
    int minNrPointsPerSegment = 10;
    float distThreshold =0.01;

    float maxDistance = 3.5f;
    Eigen::Vector4f fxycxy;
    void calcPoints();
    std::shared_ptr<gfx::GpuTex2D> segment(std::shared_ptr<gfx::GpuTex2D> dStdMaxStd,cv::Mat existingDStdMaxStd,cv::Mat existingGeometry);



};

#endif
