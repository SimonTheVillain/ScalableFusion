//
// Created by simon on 11/13/18.
//

#ifndef SUPERMAPPING_ACTIVESET_H
#define SUPERMAPPING_ACTIVESET_H

#include <iostream>
#include <memory>
#include <vector>
#include <coalescedMemoryTransfer.h>

class MapInformationRenderer;
class MapPresentationRenderer;

class MeshReconstruction;
class MeshPatch;
class MeshPatchGpuHandle;
class MeshTextureGpuHandle;
//class StitchGpuHandle;
class GpuGeomStorage;



class GpuTriangle;
template <typename T>
class GpuBufferConnector;

typedef GpuBufferConnector<GpuTriangle> TriangleBufConnector;



class ActiveSet{
    friend GpuGeomStorage;
    friend MapInformationRenderer;
    friend MapPresentationRenderer;
    friend MeshPatch;
private:
    ActiveSet(GpuGeomStorage* storage, std::vector<std::shared_ptr<MeshPatch> > patches,
            MeshReconstruction *map,
            bool initial,//TODO: also get rid of these initial frames
            bool debug1=false);

    //TODO: these:
    void UploadTexAndCoords(std::vector<std::shared_ptr<MeshPatch>> &patches,
                            std::vector<std::shared_ptr<MeshPatchGpuHandle>> &patchesGpu,
            const MeshReconstruction* map,bool initial = false);

    //void UploadTex


    void UploadTexAndCoords(MeshPatch* patch,MeshPatchGpuHandle* patchGpu, //lets check if these are necessary
                        std::vector<CoalescedGpuTransfer::Task> &coalescedTexCoordTasks);

    //TODO: these, but this seems not to be elegant
    void CheckAndAppendTriangles(const std::vector<std::shared_ptr<MeshPatch>> &patchesToCheck,
            std::vector<std::shared_ptr<MeshPatch>> &appendTo);
    void UploadTriangles(std::vector<std::shared_ptr<MeshPatch>> &patches);
    //TODO: these
    void CheckAndUpdateRefTextures(const std::vector<std::shared_ptr<MeshPatch>> &patches,MeshReconstruction* map);

    //TODO: propably the same for download

public:
    //bool wasRecentlyCreatedByExpandDELETE_DEPRECATED=false;
    std::string name;
    GpuGeomStorage* gpuGeomStorage;
    //std::vector<std::shared_ptr<MeshPatch>> toRemove;
    ~ActiveSet();

    //std::mutex vectorUpdateMutex;


    std::vector<std::shared_ptr<MeshPatch>> retainedMeshPatchesCpu;

    std::vector<std::shared_ptr<MeshPatchGpuHandle>> retainedMeshPatches;

    std::vector<std::shared_ptr<TriangleBufConnector>> retainedDoubleStitches;
    std::vector<std::shared_ptr<TriangleBufConnector>> retainedTripleStitches;//TODO: implement this (almost just for ref textures)
    std::shared_ptr<TriangleBufConnector> retainedTripleStitchesCoalesced;


    //TODO: is it better retaining it here compared to retaining it in the actual gpumesh structure?
    //std::vector<std::shared_ptr<MeshTextureGpuHandle>> retainedMeshTextureGpuHandles;


    void drawDoubleStitches();
    void drawTripleStitches();
    void drawPatches();

    void drawEverything();

    void reuploadHeaders();

    void checkForCompleteGeometry();

};



#endif //SUPERMAPPING_ACTIVESET_H
