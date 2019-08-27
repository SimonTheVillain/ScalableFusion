#ifndef FILE_TEX_COORDS_H
#define FILE_TEX_COORDS_H
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "gpuMeshStructure.h"



//class MeshPatch;
//class MeshTexture;

//better do this by triangles since we would have to select all the
class TexCoordGen{
public:
    struct Task{
        GpuTriangle* triangles;
        uint32_t triangleCount;
        Eigen::Vector2f* coords;
        float scale_x;
        float scale_y;
        float offset_x;
        float offset_y;
    };

    static void genTexCoords(std::vector<Task> tasks, Eigen::Matrix4f proj_pose,
                             GpuPatchInfo *patchInfos, GpuVertex *gpuVertices);

    /*
    static void genTexCoords(std::vector<std::shared_ptr<MeshPatch> > patches,
                             std::vector<std::shared_ptr<MeshTexture> > texPatches,
                             std::vector<cv::Rect2f> bounds,
                             Eigen::Matrix4f proj_pose,
                             GpuPatchInfo* patchInfos,
                             GpuVertex* gpuVertices);
    */

    struct BoundTask{
        GpuTriangle* triangles;
        uint32_t triangleCount;
        uint32_t targetInd;
        int debugType = 0;
    };


    static std::vector<cv::Rect2f> getPotentialTexCoordBounds(std::vector<BoundTask> tasks, Eigen::Matrix4f proj_pose, int resultCount, GpuPatchInfo *patchInfos, GpuVertex *vertices);
    /*
    static std::vector<cv::Rect2f> getPotentialTexCoordBounds(
            std::vector<std::shared_ptr<MeshPatch>> patches,Eigen::Matrix4f proj_pose);
    */
};

#endif


