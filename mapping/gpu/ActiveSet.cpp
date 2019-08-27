//
// Created by simon on 11/13/18.
//

#include "ActiveSet.h"
#include "gpuGeomStorage.h"
#include "../meshReconstruction.h"


#include "../base/meshStructure.h"
#include "../base/textureStructure.h"


#include "../cuda/gpuErrchk.h"
#include "../cuda/float16_utils.h"
#include "../cuda/texPatchInit.h"
#include "../cuda/stdTexUpdate.h"
#include "../cuda/coalescedMemoryTransfer.h"

#include <Eigen/Core>

#include <tuple>
#include <unordered_map>


using namespace std;
using namespace Eigen;


ActiveSet::ActiveSet(GpuGeomStorage *storage,
                     std::vector<std::shared_ptr<MeshPatch> > patches,
                     MeshReconstruction* map,bool initial,
                     bool debug1)
{
    //TODO: for all the patches that needed to be reuploaded we also upload the
    //textures!


    gpuGeomStorage=storage;


    vector<shared_ptr<MeshPatch>> newMeshPatchesCpu;
    vector<shared_ptr<MeshPatchGpuHandle>> newMeshPatchesGpu;
    vector<shared_ptr<TriangleBufConnector>> meshStitchesGpu;

    //combined download for triangles and vertices
    vector<CoalescedGpuTransfer::Task> coalescedVertexTasks;
    vector<GpuVertex> coalescedVertices;

    vector<CoalescedGpuTransfer::Task> coalescedTriangleTasks;
    vector<GpuTriangle> coalescedTriangles;


    //connect patches and gpu patches in a map until we are finally connecting
    unordered_map<MeshPatch*,shared_ptr<MeshPatchGpuHandle>> patchMap;




    //also combined download for the header?

    //manually upload all the new patches:
    for(size_t i = 0;i< patches.size();i++){
        //create all the necessary buffers and also upload the triangles
        MeshPatch *patch = patches[i].get();
        shared_ptr<MeshPatchGpuHandle> gpuPatch = patch->gpu.lock();

        //if the gpuPatch exists we should just secure it and will not need to download
        if(gpuPatch == nullptr){
            //if there is no gpuPatch we create a new one.
            gpuPatch =
                    make_shared<MeshPatchGpuHandle>(storage,
                                                    patch->vertices.size(),
                                                    patch->triangles.size());

            //TODO: get rid of the vertices in download thingy
            /*
            if(gpuVerticesInDownload!=nullptr){
                //assert(0);
                gpuPatch->verticesSource = gpuVerticesInDownload;
                //pls add code to reuse these currently downloaded vertices
                //this should only trigger in multithreaded mode!!!
                //also add this to the constructor
            }*/
            //patch->gpu = gpuPatch; //TODO: maybe only do this after downloading all of the patches is commenced
            newMeshPatchesGpu.push_back(gpuPatch);
            newMeshPatchesCpu.push_back(patches[i]);

            //TODO:
            //triggering this update not only when the gpu resources are
            //initially created
            //if(gpuVerticesInDownload==nullptr){//TODO: get rid of this verticesIndownloadThingy
                CoalescedGpuTransfer::Task task;
                task.count = patch->vertices.size();
                task.start = coalescedVertices.size();
                task.target = gpuPatch->verticesSource->getStartingPtr();
                coalescedVertexTasks.push_back(task);

                //create the necessary vertices
                for(size_t j=0;j<patch->vertices.size();j++){
                    coalescedVertices.push_back(patch->vertices[j].genGpuVertex());
                }
            //}

            //when reuploading the geometry before it got downloaded
            //we use these vertices to prevent the gpu content from being
            //deleted
            //patch->mostCurrentVertices = gpuPatch->verticesSource;
            //patch->mostCurrentTriangles = gpuPatch->triangles;
        }

        if(gpuPatch==nullptr){
            //at this point we really should have a gpu resource
            assert(0);
        }
        //Triangles are done further down
        //retainedMeshPatchesCpu.push_back(patches[i]);
        retainedMeshPatches.push_back(gpuPatch);
        patch->addActiveSet(this);
        patchMap[patch] = gpuPatch;

    }
    retainedMeshPatchesCpu = patches;
    CoalescedGpuTransfer::upload(coalescedVertices,coalescedVertexTasks);


    //check if really all of the gpu patches are valid
    for(size_t i=0;i<retainedMeshPatches.size();i++){
        if(retainedMeshPatches[i] == nullptr){
            assert(0);
        }
    }




    //only upload triangles for patches we uploaded the vertices
    //TODO: also for triangles with chenged position of the header and similar issues
    for(size_t i = 0;i< newMeshPatchesCpu.size();i++){
        MeshPatch *patch = newMeshPatchesCpu[i].get();
        shared_ptr<MeshPatchGpuHandle> gpuPatch = newMeshPatchesGpu[i];

        CoalescedGpuTransfer::Task task;
        task.count = patch->triangles.size();
        task.start = coalescedTriangles.size();
        task.target = gpuPatch->triangles->getStartingPtr();
        coalescedTriangleTasks.push_back(task);
        for(size_t j=0;j<patch->triangles.size();j++){
            Triangle &triangle = patch->triangles[j];
            GpuTriangle gpuTriangle;
            shared_ptr<MeshPatchGpuHandle> gpuThisPt = patchMap[triangle.points[0].getPatch()];
            for(size_t k=0;k<3;k++){
                VertexReference pr = triangle.points[k];
                shared_ptr<MeshPatchGpuHandle> gpuThisPtDebug =
                        patchMap[triangle.points[k].getPatch()];
                //TODO: do something else to fix this
                //obviously this fails if we don't set gpu references
                gpuTriangle.patchInfoInds[k] =
                        gpuThisPt->patchInfos->getStartingIndex();
                gpuTriangle.indices[k] = pr.getIndex();
                gpuTriangle.texIndices[k] = triangle.
                        texIndices[k];
            }
            coalescedTriangles.push_back(gpuTriangle);
        }
    }

    //setup lists with unique stitches of each type
    set<shared_ptr<DoubleStitch>> doubleStitches;
    set<shared_ptr<TripleStitch>> tripleStitches;
    for(size_t i=0;i<patches.size();i++){
        MeshPatch *patch = patches[i].get();


        //now add the triangles for stitches to the set:
        for(size_t j=0;j<patch->doubleStitches.size();j++){
            shared_ptr<DoubleStitch> stitch = patch->doubleStitches[j];
            MeshPatch* debugPatch1 =stitch->patches[0].lock().get();
            MeshPatch* debugPatch2 = stitch->patches[1].lock().get();
            if(stitch==nullptr){
                continue;
            }
            //check if this patch is this stitches main patch
            if(stitch->patches[0].lock().get() != patch){
                continue;
            }
            if(!stitch->isPartOfActiveSet(this)){
                continue;
            }

            shared_ptr<TriangleBufConnector> gpu = stitch->trianglesGpu.lock();
            //TODO: also check if all the dependencies are met!!!!
            if(gpu == nullptr){
                //in case the stitch is lacking a gpu representation:
                //create a new one
                gpu = storage->triangleBuffer->getBlock(stitch->triangles.size());
                stitch->trianglesGpu = gpu;
                //create a upload task:
                CoalescedGpuTransfer::Task task;
                task.count = stitch->triangles.size();
                task.start = coalescedTriangles.size();
                task.target = gpu->getStartingPtr();
                coalescedTriangleTasks.push_back(task);
                for(size_t j=0;j<stitch->triangles.size();j++){
                    Triangle &triangle = stitch->triangles[j];
                    GpuTriangle gpuTriangle;
                    for(size_t k=0;k<3;k++){
                        VertexReference pr = triangle.points[k];
                        //shared_ptr<MeshPatchGpuHandle> gpuThisPt =
                        //        pr.getPatch()->gpu.lock();
                        shared_ptr<MeshPatchGpuHandle> gpuThisPt =
                                patchMap[pr.getPatch()];
#ifdef DEBUG
                        if(gpuThisPt==nullptr){
                            //no triangle should have any invalid reference
                            assert(0);
                        }
#endif
                        gpuTriangle.patchInfoInds[k] =
                                gpuThisPt->patchInfos->getStartingIndex();
                        gpuTriangle.indices[k] = pr.getIndex();
                        gpuTriangle.texIndices[k] = triangle.texIndices[k];
                    }
                    coalescedTriangles.push_back(gpuTriangle);
                }

            }
            meshStitchesGpu.push_back(gpu);
            doubleStitches.insert(stitch);
            //if(stich_)
            //if(patch->doubleStitches[j].lock()->)
        }
        for(size_t j=0;j<patch->tripleStitches.size();j++){
            shared_ptr<TripleStitch> stitch = patch->tripleStitches[j];

            if(stitch==nullptr){
                continue;
            }
            //check if this patch is this stitches main patch
            if(stitch->patches[0].lock().get() != patch){
                continue;
            }
            if(!stitch->isPartOfActiveSet(this)){
                continue;
            }
            tripleStitches.insert(stitch);
        }
    }

    //Uploading one coalesced set of triangles and also upload singular triangles when needed
    CoalescedGpuTransfer::Task coalescedTask;
    coalescedTask.start = coalescedTriangles.size();
    for(auto tripStitch : tripleStitches){
        //no need to check the main patch and stuff since we already did this

        shared_ptr<TripleStitch> stitch = tripStitch;
        CoalescedGpuTransfer::Task task;
        task.start = coalescedTriangles.size();

        if(stitch->triangles.size()==0){
            assert(0); //a empty stitch should not exist.
        }

        for(size_t j=0;j<stitch->triangles.size();j++){
            Triangle &triangle = stitch->triangles[j];
            GpuTriangle gpuTriangle;
            for(size_t k=0;k<3;k++){
                VertexReference pr = triangle.points[k];
                //shared_ptr<MeshPatchGpuHandle> gpuThisPt =
                //        pr.getPatch()->gpu.lock();
                shared_ptr<MeshPatchGpuHandle> gpuThisPt =
                        patchMap[pr.getPatch()];
                gpuTriangle.patchInfoInds[k] =
                        gpuThisPt->patchInfos->getStartingIndex();
                gpuTriangle.indices[k] = pr.getIndex();
                gpuTriangle.texIndices[k] = triangle.texIndices[k];
            }
            coalescedTriangles.push_back(gpuTriangle);
        }
        //only create a new buffer for the triple stitch if the content changes (TODO)
        //or when there is nothing uploaded yet
        shared_ptr<TriangleBufConnector> gpu = stitch->trianglesGpu.lock();
        if(gpu == nullptr){
            task.count = coalescedTriangles.size()-task.start;
            shared_ptr<TriangleBufConnector> gpu = storage->triangleBuffer->getBlock(task.count);
            stitch->trianglesGpu = gpu;
            this->retainedTripleStitches.push_back(gpu);
            task.target = gpu->getStartingPtr();
            coalescedTriangleTasks.push_back(task);
        }else{
            this->retainedTripleStitches.push_back(gpu);
        }
    }
    coalescedTask.count = coalescedTriangles.size()-coalescedTask.start;
    //now create a new buffer!
    if(coalescedTask.count!=0){
        shared_ptr<TriangleBufConnector> gpu = storage->triangleBuffer->getBlock(coalescedTask.count);
        coalescedTask.target = gpu->getStartingPtr();
        coalescedTriangleTasks.push_back(coalescedTask);

        this->retainedTripleStitchesCoalesced = gpu;
    }

    CoalescedGpuTransfer::upload(coalescedTriangles,coalescedTriangleTasks);


    //TODO: remove these debug measures. get rid as soon as we don't need it anymore
/*********************************************************************/
/*
    for(shared_ptr<MeshPatch> patch : debugPatches){
        if(patch->gpu.expired()){
            continue;
        }
        if(patch->gpu.lock() == nullptr){
            continue;
        }

        if(patch->gpu.lock()->geomTex == nullptr){
            continue;
            assert(0);
        }

    }

    for(shared_ptr<MeshPatch> patch : patches){
        if(!debug1){
            continue;
        }
        continue;
        if(patch->gpu.lock() == nullptr){
            assert(0);
            continue;
        }

        if(patch->gpu.lock()->geomTex == nullptr){
            continue;
            assert(0);
        }

    }
    */
    /**********************************************************************/


    //retain the collected stitches in this active set
    this->retainedDoubleStitches = meshStitchesGpu;


    //DEBUG! this is destroying everything!!//TODO: is it? is it necessary? (maybe not here but further down?)
    //reuploadHeaders();//shouldn't work at this point really

    //debug
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    //return;//DEBUG: the part below this is creating the invalid points... why?

    //TODO: checkout what this is doing!!!
    vector<CoalescedGpuTransfer::Task> coalescedTexCoordTasks;
    vector<Eigen::Vector2f> coalescedTexCoords;

    /******************TEXTURE/TEXCOORDS!!!!!!!!!!!!***********************/
    //new cleaner approach
    //cout << newMeshPatchesCpu.size() << endl;
    UploadTexAndCoords(newMeshPatchesCpu,newMeshPatchesGpu,map,initial);

    //after all the data is downloaded we "atomically" set the references to the gpu
    for(size_t i = 0; i< newMeshPatchesCpu.size();i++){
        newMeshPatchesCpu[i]->gpu = newMeshPatchesGpu[i];
    }

    //uploading the header so we can update the reference textures
    reuploadHeaders();
    CheckAndUpdateRefTextures(retainedMeshPatchesCpu,map);
    //update the headers to show the new reference textures
    reuploadHeaders();
    //TODO: debug stupid bugs

    for(int i=0;i<newMeshPatchesCpu.size();i++){
        shared_ptr<MeshPatch> patch = newMeshPatchesCpu[i];
        for(int j=0;j<patch->texPatches.size();j++){
            if(patch->texPatches[j]->gpu.lock() == nullptr){
                assert(0);//this should not be here
            }
        }
    }
    for(int i=0;i<patches.size();i++){
        shared_ptr<MeshPatch> patch = patches[i];
        for(int j=0;j<patch->texPatches.size();j++){
            if(patch->texPatches[j]->gpu.lock() == nullptr){
                assert(0);//this should not be here
            }
        }
    }

}

ActiveSet::~ActiveSet()
{

    //TODO: time this! make the download enforcable to prevent it from happening during time critical tasks.

    vector<shared_ptr<MeshPatch>> patchesToBeDownloaded;
    patchesToBeDownloaded.reserve(retainedMeshPatchesCpu.size());
    for(shared_ptr<MeshPatch> patch : retainedMeshPatchesCpu){
        //remove this active set from all patches.
        if(patch->removeActiveSet(this)){
            if(patch->gpu.lock() == nullptr){
                assert(0); //the patch was owned by at least this active set. it should have a gpu representation
            }
            patchesToBeDownloaded.push_back(patch);
        }
    }

    //TODO: download the textures and vertices (texCoords?) to the according cpu structures
    vector<CoalescedGpuTransfer::DirectTask> coalescedDownloadTasks;
    vector<tuple<shared_ptr<MeshPatch>,vector<GpuVertex>>> downloadedVertices;
    vector<tuple<shared_ptr<MeshTexture>,vector<Vector2f>>> downloadedTexCoords;
    //for the beginning lets do the texture transfer uncoalesced
    for(shared_ptr<MeshPatch> patch : patchesToBeDownloaded){
        //TODO: add a retain count to the gpu handle
        //TODO: not so fast there is a active set list for each MeshPatch (remove these todos after implementation)
        shared_ptr<MeshPatchGpuHandle> patchGpu = patch->gpu.lock();

        //check if the gpu ever changed anything with the vertices
        if(patchGpu->gpuVerticesChanged){
            //and if add to the coalesced download
            CoalescedGpuTransfer::DirectTask task;
            int count = patchGpu->verticesDest->getSize();
            task.src = patchGpu->verticesDest->getStartingPtr();
            task.byteCount = sizeof(GpuVertex)*count;
            vector<GpuVertex> vertices(count);
            task.dst = &vertices[0];
            //cout << &vertices[0] << endl;//DEBUG: check the pointer of the vertices
            tuple<shared_ptr<MeshPatch>,vector<GpuVertex>> t = make_tuple(patch,std::move(vertices));
            //cout << &(get<1>(t)[0]) << endl;
            downloadedVertices.push_back(std::move(t));
            //cout << &get<1>(downloadedVertices[downloadedVertices.size()-1])[0] << endl;
            coalescedDownloadTasks.push_back(task);


        }



        //DOWNLOAD TO INTERMEDIATE VECTORS WHICH THEN SHOULD BE ATOMICLY STD::MOVE D
        //https://stackoverflow.com/questions/43243977/assigning-vectors-without-copying-them
        {
            shared_ptr<MeshTextureGpuHandle> texPatchGpu = patch->geomTexPatch->gpu.lock();
            shared_ptr<MeshTexture> texPatch = patch->geomTexPatch;
            if(texPatch == nullptr){
                assert(0); //the geometry texture should exist in any case!!!!!!!!
            }
            if(texPatchGpu == nullptr){
                assert(0);//the geometry texture should exist in any case.
            }


            if(texPatchGpu->gpuDataChanged){
                cv::Rect2i roi = texPatchGpu->tex->getRect();
                cv::Mat mat(roi.height,roi.width,CV_32FC4);
                //downloading process

                castF16SurfaceToF32Buffer(texPatchGpu->tex->getCudaSurfaceObject(),
                        roi.x,roi.y,roi.width,roi.height,
                                          (float*)mat.data,4);

                texPatch->mat = mat;
                texPatchGpu->gpuDataChanged = false;//in case somebody really safes this gpu patch last minute
            }
            CoalescedGpuTransfer::DirectTask task;
            int count = texPatchGpu->coords->getSize();
            task.src = texPatchGpu->coords->getStartingPtr();
            task.byteCount = sizeof(Vector2f) * count;
            vector<Vector2f> target(count);
            //cout << &target [0] << endl;
            task.dst = static_cast<void*>(&target[0]);
            coalescedDownloadTasks.push_back(task);
            downloadedTexCoords.push_back(make_tuple(texPatch,std::move(target)));

            //cout << &get<1>(downloadedTexCoords[downloadedTexCoords.size()-1])[0] << endl;
            //cout << "shit why aren't these pointers the same" << endl;

        }

        //download color textures
        for(size_t i=0;i<patch->texPatches.size();i++){

            shared_ptr<MeshTexture> texPatch = patch->texPatches[i];
            shared_ptr<MeshTextureGpuHandle> texPatchGpu = texPatch->gpu.lock();
            if(texPatch == nullptr){
                assert(0); //the texture should exist if it is in this list
            }
            if(texPatchGpu == nullptr){
                assert(0);//the gpu texture should exist if it is in this list
            }


            if(texPatchGpu->gpuDataChanged){
                cv::Rect2i roi = texPatchGpu->tex->getRect();
                cv::Mat mat(roi.height,roi.width,CV_8UC4);
                //downloading process
                texPatchGpu->tex->downloadData(mat.data);
                texPatch->mat = mat;
                texPatchGpu->gpuDataChanged=false;//in case somebody really safes this gpu patch last minute
            }
            CoalescedGpuTransfer::DirectTask task;
            int count = texPatchGpu->coords->getSize();
            task.src = texPatchGpu->coords->getStartingPtr();
            task.byteCount = sizeof(Vector2f) * count;
            vector<Vector2f> target(count);
            task.dst = static_cast<void*>(&target[0]);
            coalescedDownloadTasks.push_back(task);
            downloadedTexCoords.push_back(make_tuple(texPatch,std::move(target)));

        }

        //TODO: download label textures

        //OBVIOUSLY ALWAYS CHECK IF THERE IS A DOWNLOADWORTHY UPDATE

        //set the retain count to zero

        //if necessary setup a coalesced memory transfer for the vertices

        //same for the texture

        //same for texCoords

    }
    //return;
    //execute the download
    CoalescedGpuTransfer::download(coalescedDownloadTasks);


    //copy the buffer over to the according elements
    //vertices
    for(size_t i = 0; i< downloadedVertices.size();i++){
        shared_ptr<MeshPatch> patch = get<0>(downloadedVertices[i]);
        vector<GpuVertex> &vertsGpu = get<1>(downloadedVertices[i]);
        assert(vertsGpu.size() == patch->vertices.size());
        //TODO: secure this with a mutex!!!!
        for(size_t j=0;j<patch->vertices.size();j++){
            patch->vertices[j].n = vertsGpu[j].n;
            patch->vertices[j].p = vertsGpu[j].p;
        }
        //vector<Vertex> verts(vertsGpu.size());
        //for(size_t j= 0;j<verts.size();j++){
        //    verts[j] = vertsGpu[j];
        //}
        //TODO: maybe mutex here
        //patch->vertices = std::move(verts);

        //TODO: secure this with a mutex!!!!

    }
    //texcoords
    for(size_t i=0;i<downloadedTexCoords.size();i++){

    }
    for(auto t : downloadedTexCoords){
        //TODO: maybe we also want to use mutexes here and stuff
        get<0>(t)->texCoords = std::move(get<1>(t));
    }



}


//ideally this method only got patches with newly generated gpuPatches but probably it doesn't have any gpuTextures
void ActiveSet::UploadTexAndCoords(std::vector<std::shared_ptr<MeshPatch>> &patches,
                                    vector<shared_ptr<MeshPatchGpuHandle>> &patchesGpu,
        const MeshReconstruction* map,bool initial) {


    vector<CoalescedGpuTransfer::Task> coalescedTexCoordTasks;
    vector<Eigen::Vector2f> coalescedTexCoords;

    for(size_t k = 0; k< patches.size();k++){
        shared_ptr<MeshPatch> patch = patches[k];
        shared_ptr<MeshPatchGpuHandle> patchGpu = patchesGpu[k];
        if(patchGpu == nullptr){
            assert(0); //the gpuPatch should exist at this point
        }

        shared_ptr<MeshTexture> texPatch = patch->geomTexPatch; //TODO: here was a crash... why is texPatch zero
        if(!texPatch->mat.empty()){//if it is empty there is nothing to do here!!!!
            //but if there is content upload it to the gpu
            //TODO: ideally this should be 1/2 lines
            if(patchGpu->geomTex!=nullptr){
                assert(0);//as all of these patches are reuploads this geomTex should not exist
            }

            int width = texPatch->mat.cols;
            int height = texPatch->mat.rows;

            shared_ptr<MeshTextureGpuHandle> texPatchGpu =
                    make_shared<MeshTextureGpuHandle>(
                            gpuGeomStorage->texPosBuffer,
                            texPatch->texCoords.size(),
                            map->texAtlasGeomLookup.get(),
                            map->texAtlasStds.get(),//TODO: the references are supposed to be filled at "CheckAndUpdateRefTextures"
                            width,height);

            //now do the uploading
            cv::Rect2i roi = texPatchGpu->tex->getRect();

            //upload the texture:
            //maybe we should also hold the texture with a mutex
            cudaSurfaceObject_t surface =
                    texPatchGpu->tex->getCudaSurfaceObject();
            //TODO: this definitely is something that we could concatenate
            castF32BufferToF16Surface(surface,
                                      roi.x,roi.y,
                                      roi.width,roi.height,
                                      (float*)texPatch->mat.data,4);

            //create the task for the tex coord upload

            if(texPatch->texCoords.size() == 0){
                assert(0);//either the tex coords would reside on gpu (then we wouldn't reach this code.
                //or they are on cpu (in which case we wouldn't reach this assert.
            }
            CoalescedGpuTransfer::Task task;
            task.count = texPatch->texCoords.size();
            task.target = texPatchGpu->coords->getStartingPtr();
            task.start = coalescedTexCoords.size();
            coalescedTexCoordTasks.push_back(task);
            coalescedTexCoords.insert(coalescedTexCoords.end(),
                                      texPatch->texCoords.begin(),
                                      texPatch->texCoords.end());




            //store/secure the texture
            patchGpu->geomTex = texPatchGpu;
            texPatch->gpu = texPatchGpu;
        }else{
            if(!initial){
                assert(0);//the mesh patch has to have either geometry data on gpu or on cpu
            }
        }

        for(size_t i=0; i < patch->texPatches.size();i++){
            texPatch = patch->texPatches[i];
            if(!texPatch->mat.empty()){
                shared_ptr<MeshTextureGpuHandle> texPatchGpu = patchGpu->texs[i];
                if(texPatchGpu!=nullptr){
                    assert(0); // we should only be doing this if the gpu texture is zero
                }
                if(texPatch->mat.empty()){
                    assert(0);//this really should not be empty
                }
                int width = texPatch->mat.cols;
                int height = texPatch->mat.rows;
                texPatchGpu =
                        make_shared<MeshTextureGpuHandle>(
                                gpuGeomStorage->texPosBuffer,
                                texPatch->texCoords.size(),
                                nullptr,
                                map->texAtlasRgb8Bit.get(),//TODO: where to get these from?
                                width,height);


                texPatch->matMutex.lock();
                patch->texPatches[i]->gpu = texPatchGpu;
                texPatchGpu->tex->uploadData(texPatch->mat.data);
                texPatch->matMutex.unlock();

                //tex coordinate upload
                CoalescedGpuTransfer::Task task;
                task.count = texPatch->texCoords.size();
                task.start = coalescedTexCoords.size();
                task.target = texPatchGpu->coords->getStartingPtr();
                coalescedTexCoordTasks.push_back(task);
                coalescedTexCoords.insert(coalescedTexCoords.end(),
                                          texPatch->texCoords.begin(),
                                          texPatch->texCoords.end());


                patchGpu->texs[i] = texPatchGpu;
                texPatch->gpu = texPatchGpu;


            }else{
                assert(0); // if the texture is not filled it should not exist in this list
            }
        }

        texPatch = patch->labelTexPatch;
        if(texPatch != nullptr){
            //also do it for the label texture
            if(!texPatch->mat.empty()){
                //TODO:
            }else{
                //it would be ok if there is no label data
            }

        }
    }

    //concatenated upload of tex coords
    CoalescedGpuTransfer::upload(coalescedTexCoords,coalescedTexCoordTasks);




}

void ActiveSet::CheckAndUpdateRefTextures(const std::vector<std::shared_ptr<MeshPatch>> &patches,
        MeshReconstruction* map) {
    vector<shared_ptr<MeshPatch>> datedPatches;
    vector<shared_ptr<MeshTexture>> datedTextures;

    for(size_t i = 0;i<patches.size();i++){
        shared_ptr<MeshPatch> patch = patches[i];
        if(!patch->isPartOfActiveSetWithNeighbours(this)){
            continue;
        }
        shared_ptr<MeshPatchGpuHandle> patchGpu = patch->gpu.lock();
        shared_ptr<MeshTextureGpuHandle> texPatchGpu = patchGpu->geomTex;
        if(texPatchGpu != nullptr){
            if(!texPatchGpu->checkRefTexDependencies()){
                datedPatches.push_back(patch);
                datedTextures.push_back(patch->geomTexPatch);
            }
        }

        //TODO: also update refTextures for the labelTextures if available

    }


    map->texturing.GenLookupTex(this,datedPatches,datedTextures,true);//true: dilate the resulting textures

}

void ActiveSet::drawDoubleStitches()
{
    for(size_t i=0;i<retainedDoubleStitches.size();i++){
        TriangleBufConnector &currentStitch = *retainedDoubleStitches[i];
        //debug start:
        //GpuTriangle triangles[currentStitch.count];
        //currentStitch.triangles->download(triangles);
        //debug end
        int slot =
                static_cast<int>(currentStitch.getStartingIndex());
        int count =
                static_cast<int>(currentStitch.getSize());
        glDrawArrays(GL_TRIANGLES,
                     slot*3,
                     count*3);
    }
}

void ActiveSet::drawTripleStitches()
{
    if(retainedTripleStitchesCoalesced ==nullptr){
        return;
    }
    TriangleBufConnector &currentStitch = *retainedTripleStitchesCoalesced;

    //debug start:
    //GpuTriangle triangles[currentStitch.count];
    //currentStitch.triangles->download(triangles);
    //debug end

    int slot = currentStitch.getStartingIndex();
    int count = currentStitch.getSize();
    glDrawArrays(GL_TRIANGLES,
                 slot*3,
                 count*3);
}

void ActiveSet::drawPatches()
{
    //cout << "rendering " << activeSet->currentlyAdded.size() << " meshPatches" << endl;
    //first iterate over all the mesh patches:
    for(size_t i=0;i<retainedMeshPatches.size();i++){
        MeshPatchGpuHandle &currentPatch =
                *retainedMeshPatches[i];

        //first check that all the textures are resident
        if(currentPatch.geomTex!=nullptr){
            currentPatch.geomTex->tex->getTex()->makeResidentInThisThread();
            currentPatch.geomTex->refTex->getTex()->makeResidentInThisThread();
        }
        for(size_t j=0;j<currentPatch.texCount;j++){
            if(currentPatch.texs[j]!=nullptr){
                currentPatch.texs[j]->tex->getTex()->makeResidentInThisThread();
            }
        }
        //now do the real rendering:
        //TODO: maybe put this code into the active set class.

        int slot = currentPatch.triangles->getStartingIndex();
        int count = currentPatch.triangles->getSize();
        glDrawArrays(GL_TRIANGLES,
                     slot*3,
                     count*3);
    }
}

void ActiveSet::drawEverything()
{

    drawPatches();
    drawDoubleStitches();
    drawTripleStitches();


}

void ActiveSet::reuploadHeaders()
{
    //TODO: also fill header data for the
    //assert(0);
    /*
    cout << "[ActiveSet::reuploadHeaders] "
            "This function is not fully implemented yet" << endl;
    */
    vector<GpuPatchInfo> coalescedInfos;
    vector<GpuPatchInfo*> coalescedInfoPos;
    for(shared_ptr<MeshPatch> patch : retainedMeshPatchesCpu){
        GpuPatchInfo info;
        shared_ptr<MeshPatchGpuHandle> gpu = patch->gpu.lock();
        info.patchId = patch->id;
        info.debug1 = patch->debug1;

        //put on the texturing information:
        if(gpu->geomTex !=nullptr){
            info.stdTexture = gpu->geomTex->genTexInfo();

            info.stdTexture.glRefTexPtrDEBUG =
                    gpu->geomTex->refTex->getGlHandle();
            cv::Rect2i roi =
                    gpu->geomTex->refTex->getRect();
            info.stdTexture.refTexPosDEBUG = Vector2f(roi.x,roi.y)*(1.0f/1024.0f);

            //debug
            //info.stdTexture.glRefTexPtrDEBUG=info.stdTexture.glTexPointer;
        }else{
            info.stdTexture.texCoordStartInd=0;
            info.stdTexture.glTexPointer = 0;
        }



        info.texLayers=0;
        patch->texPatchesMutex.lock();
        for(size_t i=0;i<patch->texPatches.size();i++){
            shared_ptr<MeshTextureGpuHandle> gpuTexPatch =
                    patch->texPatches[i]->gpu.lock();
            if(gpuTexPatch==nullptr){
                //this really should not happen.... so why is this?
                //TODO: place assert here and check
                continue;
            }
            info.textureInfos[info.texLayers] = gpuTexPatch->genTexInfo();
                    //gpuTexPatch->genSrcTexInfo();
            info.texLayers++;
        }
        patch->texPatchesMutex.unlock();
        //TODO. this texture upload



        //the labelling textures:
        patch->labelTexPatchMutex.lock();
        if(patch->labelTexPatch != nullptr){
            shared_ptr<MeshTextureGpuHandle> gpuTexPatch =
                    patch->labelTexPatch->gpu.lock();
            if(gpuTexPatch==nullptr){
                //this really should not happen.... so why is this?
                //TODO: place assert here and check
                assert(0);
                continue;
            }
            info.segmentationTexture = gpuTexPatch->genTexInfo();//tex->genTexInfo();
            info.segmentationTexValid = true;
        }
        // = static_cast<int32_t>(labelCount);
        patch->labelTexPatchMutex.unlock();


        info.vertexSourceStartInd = gpu->verticesSource->getStartingIndex();
        info.vertexDestinationStartInd = gpu->verticesDest->getStartingIndex();
        info.triangleStartInd = gpu->triangles->getStartingIndex();




        coalescedInfos.push_back(info);
        coalescedInfoPos.push_back(gpu->patchInfos->getStartingPtr());
    }

    CoalescedGpuTransfer::upload(coalescedInfos,coalescedInfoPos);
}

void ActiveSet::checkForCompleteGeometry()
{

    for(size_t i=0;i<retainedMeshPatches.size();i++){
        if(retainedMeshPatches[i]==nullptr){
            assert(0);
        }
        if(retainedMeshPatchesCpu[i]->geomTexPatch == nullptr){
            assert(0); // something is very fishy
        }
        if(retainedMeshPatches[i]->geomTex==nullptr){
            //assert(0);
        }
    }
}

