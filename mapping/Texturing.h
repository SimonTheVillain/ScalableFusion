//
// Created by simon on 8/2/19.
//

#ifndef SUPERMAPPING_TEXTURING_H
#define SUPERMAPPING_TEXTURING_H


#include <memory>
#include <Eigen/Eigen>
#include "base/meshStructure.h"


class MeshReconstruction;

class Texturing {
public:
    MeshReconstruction *meshReconstruction;



    //formerly
    void GenerateGeomTex(std::vector<std::shared_ptr<MeshPatch> > &newPatches,
                         Eigen::Matrix4f pose, Eigen::Matrix4f proj,
                         std::shared_ptr<gfx::GpuTex2D> geomSensorData,
                         std::shared_ptr<ActiveSet> activeSet);


    void ProjToGeomTex(ActiveSet* activeSet, std::vector<std::shared_ptr<MeshPatch> > &newPatches,
                       std::shared_ptr<gfx::GpuTex2D> geomSensorData,
                       Eigen::Matrix4f pose, Eigen::Matrix4f proj);


    //TODO: put this to geometryUpdate else or split it up properly
    void VertGeomTexUpdate(std::shared_ptr<gfx::GpuTex2D> dStdTex,
                           Eigen::Matrix4f depthPoseIn,
                           std::shared_ptr<ActiveSet> &activeSet);
    void ColorTexUpdate(std::shared_ptr<gfx::GpuTex2D> rgbaTex,
                        Eigen::Matrix4f colorPoseIn,
                        std::shared_ptr<ActiveSet> &activeSet);

    void ApplyColorData(std::vector<std::shared_ptr<MeshPatch>> &visiblePatches,
                           std::shared_ptr<gfx::GpuTex2D> rgbIn,
                           Eigen::Matrix4f &pose, Eigen::Matrix4f &proj, std::shared_ptr<ActiveSet> activeSet);

    void GenLookupTexGeom(ActiveSet *activeSet, std::vector<std::shared_ptr<MeshPatch> > &patches);
    void GenLookupTex(ActiveSet *activeSet,
                      std::vector<std::shared_ptr<MeshPatch> > &patches,
                      std::vector<std::shared_ptr<MeshTexture>> &textures,
                      bool dilate = true);

};


#endif //SUPERMAPPING_TEXTURING_H
