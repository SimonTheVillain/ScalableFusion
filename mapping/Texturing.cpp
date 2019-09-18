//
// Created by simon on 8/2/19.
//

#include "Texturing.h"
#include "cuda/texCoords.h"
#include "cuda/texPatchInit.h"
#include "cuda/stdTexUpdate.h"
#include "gpu/ActiveSet.h"
#include "debugRender.h"
#include "meshReconstruction.h"

#include "camera.h"

using namespace Eigen;
using namespace std;
using namespace cv;

void Texturing::GenerateGeomTex(std::vector<std::shared_ptr<MeshPatch> > &newPatches,
                                                        Eigen::Matrix4f pose, Eigen::Matrix4f proj,
                                                        shared_ptr<gfx::GpuTex2D> geomSensorData,
                                                        std::shared_ptr<ActiveSet> activeSet)
{
    MeshReconstruction* mesh = meshReconstruction;
    //assert(0);//is this still used
    //TODO: even though commented out this still holds true
    /*
    cout << "[ScaleableMap::generateGeomTexForNovelPatches] "
            "i fear tex coordinates are still missing for vertices "
            "without triangles! This is going to be a problem with the "
            "texture update" << endl;
    */
    float scale = 2;//2;
    //cout << "[ScaleableMap::generateGeomTexForNovelPatches] update this" << endl;

    Matrix4f _pose=pose.inverse();
    Matrix4f mvp = proj*_pose;


    //all of these patches have to be valid...
    vector<shared_ptr<MeshPatch>> validMeshPatches = newPatches;

    for(shared_ptr<MeshPatch> patch : newPatches){
        for(shared_ptr<DoubleStitch> stitch : patch->double_stitches){

            if(stitch->patches[1].lock()->gpu.lock() == nullptr){
                assert(0);
            }
            if(stitch->patches[0].lock() != patch){
                continue;
            }
            if(!stitch->isPartOfActiveSet(activeSet.get())) {
                stitch->isPartOfActiveSet(activeSet.get());
                assert(0);
            }
        }
        for(shared_ptr<TripleStitch> stitch : patch->triple_stitches){

            for(int i=1;i<3;i++){
                if(stitch->patches[i].lock()->gpu.lock() == nullptr){
                    assert(0);
                }
            }
            if(stitch->patches[0].lock() != patch){
                continue;
            }
            if(!stitch->isPartOfActiveSet(activeSet.get())) {
                stitch->isPartOfActiveSet(activeSet.get());
                assert(0);
                // One explanation could be that we just do not care for loading the neighbouring patch
                // into memory even though we should.

            }
        }
    }
    std::vector<cv::Rect2f> bounds =
            mesh->genBoundsFromPatches(newPatches,pose,proj,activeSet);

    for(size_t i = 0; i< bounds.size();i++){

        Rect2f bound = bounds[i];
        //cout << bound << endl;

        if(max(bound.width,bound.height) > 1024){
            cout << "maybe download everything related to these bounds. we need to find out what is going on here" << endl;
            shared_ptr<MeshPatchGpuHandle> gpu = newPatches[i]->gpu.lock();
            int count = gpu->vertices_source->getSize();
            GpuVertex vertices[count];
            gpu->vertices_source->download(vertices);

            for(size_t j=0;j<count;j++) {
                cout << vertices[j].p << endl;

            }



            cv::namedWindow("test test");
            thatOneDebugRenderingThingy->vertex_buffer = mesh->m_gpuGeomStorage.vertexBuffer->getGlName();
            thatOneDebugRenderingThingy->info_buffer = mesh->m_gpuGeomStorage.patchInfoBuffer->getGlName();
            thatOneDebugRenderingThingy->triangle_buffer = mesh->m_gpuGeomStorage.triangleBuffer->getGlName();
            thatOneDebugRenderingThingy->tex_pos_buffer = mesh->m_gpuGeomStorage.texPosBuffer->getGlName();
            //thatOneDebugRenderingThingy->setPatch(newPatches[i].get());
            //thatOneDebugRenderingThingy->setIndexCount(gpu->triangles->getStartingIndex(),gpu->triangles->getSize());
            thatOneDebugRenderingThingy->addPatch(newPatches[i].get(),1,0,0);

            shared_ptr<MeshPatch> debugPatch = newPatches[i];
            for(int i= 0 ; i< debugPatch->double_stitches.size();i++){
                if(debugPatch->double_stitches[i]->patches[0].lock() != debugPatch){
                    continue;
                }
                if(debugPatch->double_stitches[i]->patches[1].lock()->gpu.lock() == nullptr){
                    assert(0);
                }
                thatOneDebugRenderingThingy->addPatch(debugPatch->double_stitches[i]->patches[1].lock().get(),
                                                      0,0,1);


            }


            //render the neighbouring patches.
            while(true){
                // After setting up the debug rendering for this, we wait so the user can take a look at it
                //cv::waitKey();

            }
            assert(0);
        }

    }

    if(bounds.size() != validMeshPatches.size()){
        assert(0);
    }


    std::vector<TexCoordGen::Task> texGenTasks;
    texGenTasks.reserve(validMeshPatches.size());//reserve

    vector<shared_ptr<MeshPatch>> meshPatchesWithNovelTex;
    meshPatchesWithNovelTex.reserve(validMeshPatches.size());//reserve

    for(size_t i=0;i<validMeshPatches.size();i++){
        shared_ptr<MeshPatch> patch = validMeshPatches[i];
        shared_ptr<MeshPatchGpuHandle> gpuPatch = patch->gpu.lock();

        shared_ptr<MeshTextureGpuHandle> gpuTexture = gpuPatch->geom_tex;
        if(gpuTexture==nullptr){
            //create the gpu resources if they are not existant
            int nrCoords = patch->geom_tex_patch->tex_coords.size();
            if(bounds[i].width*scale < 0){
                //why are the bounds at this one negative?
                //genBoundsFromPatches(newPatches,pose,proj,activeSet);
                continue;//do not continue with creating a new texture here!
                assert(0);//the bounds failed
            }
            //this is where we get the size
            gpuTexture =
                    make_shared<MeshTextureGpuHandle>(
                            mesh->m_gpuGeomStorage.texPosBuffer,
                            nrCoords,
                            mesh->texAtlasGeomLookup.get(),
                            mesh->texAtlasStds.get(),
                            int(bounds[i].width*scale),
                            int(bounds[i].height*scale));
            gpuPatch->geom_tex = gpuTexture;
            patch->geom_tex_patch->gpu = gpuTexture;

            //mark the texture as the most current source for data
            //in case the container gets deleted.
            //patch->geomTexPatch->texCoordsGpu = gpuTexture->coords;
            patch->geom_tex_patch->gpu = gpuTexture;
            gpuTexture->gpu_data_changed = true;

        }else{
            assert(0);//debug: i don't think the gpu texture should be existing
            //already since this is part of the creation process of novel
            //patches
        }



        //allocate textures and coordinates if necessary.
        //+ create new tasks
        TexCoordGen::Task task;
        task.offset_x = bounds[i].x-0.5f/float(scale);
        task.offset_y = bounds[i].y-0.5f/float(scale);
        task.scale_x = 1.0f/(bounds[i].width + 1.0f/float(scale));
        task.scale_y = 1.0f/(bounds[i].height + 1.0f/float(scale));
        task.coords = gpuTexture->coords->getStartingPtr();
        task.triangleCount = gpuPatch->triangles->getSize();

        task.triangles = gpuPatch->triangles->getStartingPtr();
        texGenTasks.push_back(task);

        //now do all the stitches and so on:

        for(shared_ptr<DoubleStitch> stitch : patch->double_stitches){
            if(stitch->patches[0].lock() != patch){
                continue;
            }
            if(!stitch->isPartOfActiveSet(activeSet.get())){

                //TODO: we really need to figure out why this is happening
                //the stitch must have both neighbours being part of this
                stitch->isPartOfActiveSet(activeSet.get());

#ifndef IGNORE_SERIOUS_BUG_5
                assert(0);//This actually should not happen
#endif
                continue;
            }
            shared_ptr<TriangleBufConnector> gpuStitch = stitch->triangles_gpu.lock();
            if(gpuStitch == nullptr){
                assert(0);
                continue;
            }



            task.triangleCount = gpuStitch->getSize();
            task.triangles = gpuStitch->getStartingPtr();
            texGenTasks.push_back(task);
        }

        //also triple stitches
        for(shared_ptr<TripleStitch> stitch : patch->triple_stitches){
            if(stitch->patches[0].lock() != patch){
                continue;
            }
            if(!stitch->isPartOfActiveSet(activeSet.get())){
                //TODO: we really need to figure out why this is happening
                assert(0); // again, i don't think this should happen
                continue;
            }
            shared_ptr<TriangleBufConnector> gpuStitch =
                    stitch->triangles_gpu.lock();
            //shared_ptr<StitchGpuHandle> gpuStitch = stitch->gpu.lock();{
            if(gpuStitch == nullptr){
                assert(0);
                continue;
            }

            task.triangleCount = gpuStitch->getSize();
            task.triangles = gpuStitch->getStartingPtr();
            texGenTasks.push_back(task);

        }
    }


    //execute the tasks on the gpu:

    TexCoordGen::genTexCoords(texGenTasks,mvp,
                              mesh->m_gpuGeomStorage.patchInfoBuffer->getCudaPtr(),
                              mesh->m_gpuGeomStorage.vertexBuffer->getCudaPtr());




    activeSet->reuploadHeaders();
    //we need headers pointing at the vertices

    GenLookupTexGeom(activeSet.get(), validMeshPatches);
    //this seems to have worked perfectly


    ProjToGeomTex(activeSet.get(),
                      validMeshPatches,
                      geomSensorData,
                      pose,
                      proj);





    //since the texture has data that needs to only exists on the gpu,
    //we setup the texture to be downloaded as soon
    for(shared_ptr<MeshPatch> patch : validMeshPatches){
        shared_ptr<MeshTextureGpuHandle> tex = patch->gpu.lock()->geom_tex;
        if(tex==nullptr){
            assert(0);
        }
        //tex->downloadToWhenFinished = patch->geomTexPatch;
        //tex->recycler = this->recycler;
        tex->gpu_data_changed = true;
        patch->geom_tex_patch->debug_is_uninitialized = false;
    }
    //TODO: now initialize that stuff



    //TODO: this actually should be capable of creating the ref textures
    //as well as the initial content of the textures.
    //pls!!!! do this here


    //and after doing this we can update the patch header


    activeSet->reuploadHeaders();



}


void Texturing::ProjToGeomTex(ActiveSet* activeSet, std::vector<std::shared_ptr<MeshPatch> > &newPatches,
                   std::shared_ptr<gfx::GpuTex2D> geomSensorData,
                   Eigen::Matrix4f pose, Eigen::Matrix4f proj){

    //we create a list of commands for the gpu to follow to update the textures.
    vector<InitDescriptor> commands;

    for(size_t i=0;i<newPatches.size();i++){
        MeshPatch *patch = newPatches[i].get();
        InitDescriptor command;
        shared_ptr<MeshTextureGpuHandle>  geomTexGpuHandle = patch->geom_tex_patch->gpu.lock();


        cv::Rect2i rect = geomTexGpuHandle->tex->getRect(); //getRect() //This is different from get rect
        command.out_offset = cv::Point2i(rect.x,rect.y);
        rect = geomTexGpuHandle->ref_tex->getRect();
        command.ref_offset = cv::Point2i(rect.x,rect.y);
        command.width = rect.width;
        command.height = rect.height;
        /*command.x = rect.x;
        command.y = rect.y;*/
        command.output =
                geomTexGpuHandle->tex->getCudaTextureObject();
        command.reference_texture =
                geomTexGpuHandle->ref_tex->getCudaTextureObject();
        commands.push_back(command);




    }


    //cout << "pose" << endl << pose << endl;
    //cout << "proj" << endl << proj << endl;
    //run the kernel
    Matrix4f _pose=pose.inverse();
    Matrix4f p_p=proj*_pose;
    //TODO: get rid of this scaling?
    Matrix4f scale;
    float w=geomSensorData->getWidth();
    float h=geomSensorData->getHeight();
    scale << 1.0f/w, 0, 0, 0,
            0, 1.0f/h, 0, 0,
            0, 0, 1.0f, 0,
            0, 0, 0, 1.0f;


    //cout << "mvp gpu" << proj*_pose << endl;
    stdTexInit(geomSensorData->getCudaTextureObject(),commands,
               scale*p_p,
               (GpuVertex*)activeSet->gpuGeomStorage->vertexBuffer->getCudaPtr(),
               (Vector2f*)activeSet->gpuGeomStorage->texPosBuffer->getCudaPtr(),
               (GpuTriangle*)activeSet->gpuGeomStorage->triangleBuffer->getCudaPtr(),
               (GpuPatchInfo*)activeSet->gpuGeomStorage->patchInfoBuffer->getCudaPtr());


    //TODO: this is not fully filling the textures. Get to the root of this issue

    //cout << "[ScaleableMap::projGeomToGeomTex] this leaves some holes in the texture....."
    //        " get to the root of this" << endl;



    return;//DEBUG
/*
    for(size_t i=0;i<newPatches.size();i++){
        Rect2i r = newPatches[i]->geomTexPatch->getLookupRect();
        cv::Mat testLookup(r.height,r.width,CV_32FC4);
        newPatches[i]->geomTexPatch->getLookupTexPatch()->
                downloadData(testLookup.data,r.size());

        cv::imshow("currentGeomLookupTexture",testLookup);
        //simple debug output
        cv::Mat test(r.height,r.width,CV_32FC4);

        r = newPatches[i]->geomTexPatch->getSourceRect();
        cudaSurfaceObject_t surface =
            newPatches[i]->geomTexPatch->getSourceTexPatch()->
                getTex()->getCudaSurfaceObject();
        castF16SurfaceToF32Buffer(surface,
                                  r.x,r.y,
                                  r.width,r.height,
                                  (float*)test.data,
                                  4);

        cv::imshow("currentGeomTexture",test);
        cv::waitKey();
    }
*/

}


void Texturing::ColorTexUpdate(std::shared_ptr<gfx::GpuTex2D> rgbaTex,
                    Eigen::Matrix4f colorPoseIn,
                    std::shared_ptr<ActiveSet> &activeSet){
    //cout << "ScaleableMap::colorTexUpdate DEBUG: of course this is not implemented yet" << endl;
    //return;
    auto start = std::chrono::system_clock::now();

    int width=rgbaTex->getWidth();
    int height=rgbaTex->getHeight();

/*

    //because i don't know where else to put it: add a texture here:
    //gfx::GpuTex2D texture3(GL_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,640,480,false,test.data);
    Mat rgba;
    cv::cvtColor(colorIn,rgba,CV_BGR2RGBA);
    //gfx::GpuTex2D texture2(G_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,rgba.cols,rgba.rows,false,rgba.data);
    std::shared_ptr<gfx::GpuTex2D> texture = std::make_shared<gfx::GpuTex2D>(GL_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,rgba.cols,rgba.rows,
                                                                             true,rgba.data);
    */

    uint64_t texPtr = rgbaTex->getGlHandle();
    rgbaTex->makeResidentInThisThread();
    //2. Step should be the incorporation of new sensor data into the already existing map.
    Matrix4f proj1 = Camera::genProjMatrix(meshReconstruction->params.rgbfxycxy);

    /**
     * TODO:
     * .) iterate over all visible patches
     * .) and over the color textures attached to these patches
     * .) check if the texture we are using is a better fit than the one already in place
     * .) replace or add the texture.
     *   *) calculate new tex coordinates
     *   *) normalize them and create the cutouts from the texture
     *
     */


    vector<shared_ptr<MeshPatch>> visibleSharedPatches;
    if(activeSet!=nullptr){
        visibleSharedPatches = activeSet->retainedMeshPatchesCpu;
    }

    //todo: replace the depth pose with the rgb camera pose

    //Matrix4f stupidDebug=colorPoseIn.inverse();

    vector<shared_ptr<MeshPatch>> fullyLoadedVisiblePatches;
    for(shared_ptr<MeshPatch> patch : visibleSharedPatches){
        if(patch->isPartOfActiveSetWithNeighbours(activeSet.get())){
            fullyLoadedVisiblePatches.push_back(patch);
        }
    }


    ApplyColorData(fullyLoadedVisiblePatches,rgbaTex,
                      colorPoseIn,//stupidDebug
                      proj1,
                      activeSet);//the active set all the newly created textures will be attached to



    meshReconstruction->cleanupGlStoragesThisThread();
    //fboStorage.cleanupThisThread();


    auto end = std::chrono::system_clock::now();
    /*cout << "[ScaleableMap::colorTexUpdate] time consumed for the whole texture update :" <<
            std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() <<
            "ms" << endl;*/



}



void Texturing::ApplyColorData(std::vector<shared_ptr<MeshPatch>> &visiblePatches,
                                           std::shared_ptr<gfx::GpuTex2D> rgbIn,
                                           Eigen::Matrix4f &pose, Eigen::Matrix4f &proj,
                                           std::shared_ptr<ActiveSet> activeSet)
{
    MeshReconstruction* mesh = meshReconstruction;
    if(activeSet==nullptr){
        return;
    }

    std::vector<Rect2f> bounds =
            mesh->genBoundsFromPatches(visiblePatches,pose,proj,activeSet);


    Vector4f camPos4 = pose*Vector4f(0,0,0,1);
    Vector3f camPos(camPos4[0],camPos4[1],camPos4[2]);

    int width = rgbIn->getWidth();
    float _width=1.0f/float(width);
    int height = rgbIn->getHeight();
    float _height = 1.0f/float(height);
    Matrix4f _pose=pose.inverse();
    Matrix4f mvp = proj*_pose;

    //this retains the textures that are being replaced so nothing flickers
    //because textures are released before new ones are created.
    vector<shared_ptr<MeshTextureGpuHandle>> meshTexturesBeingReplaced;

    vector<TexCoordGen::Task> texGenTasks;

    vector<CopyDescriptor> copies;

    vector<shared_ptr<MeshPatch>> patchesWithColorUpdates;

    struct GpuCpuTexPair{
        shared_ptr<MeshTexture> cpu;
        shared_ptr<MeshTextureGpuHandle> gpu;
    };
    vector<GpuCpuTexPair> updatedTextures;
    //TODO: use this for enabling RGB texture download
    //cout << "TODO: use this for enabling RGB texture download" << endl;

    for(size_t i=0;i<visiblePatches.size();i++){
        Rect2f bound = bounds[i];
        if(bound.x<0 || bound.y<0 ||
                                bound.x+bound.width>(width-1) ||
                   bound.y+bound.height>(height-1) ){
            //if patch does not have valid points something went wrong
            //most likely there are no triangles in said patch....
            //assert(0);//
            continue;
        }
        shared_ptr<MeshPatch> patch = visiblePatches[i];
        shared_ptr<MeshPatchGpuHandle> gpuPatch = patch->gpu.lock();
        vector<shared_ptr<MeshTexture>> texPatchesToDelete;
        patch->tex_patches_mutex.lock();


        //i don't trust the patch position
        float dist = (patch->getPos() - camPos).norm();

        bool createThisTexture=false;
        if(patch->tex_patches.size()!=0){
            //vector with textures that need to be removed
            for(size_t j=0;j<patch->tex_patches.size();j++){
                //iterate over all texture patches to see which of them need to be removed


                Vector4f camPosAtCapture4  = patch->tex_patches[j]->cam_pose.inverse()*Vector4f(0,0,0,1);
                Vector3f camPosAtCapture = camPosAtCapture4.block<3,1>(0,0);


                float distAtCapture = (patch->getPos() - camPosAtCapture).norm();
                if(distAtCapture*mesh->params.maxDepthFactorThreshForTexAdding > dist){
                    //now the camera is so close that the new texture is of
                    //way higher quality. It is time to remove the old texture
                    createThisTexture = true;
                    texPatchesToDelete.push_back(patch->tex_patches[j]);
                    //cout << "[ScaleableMap::applyNewColorData] Replacing an existing texture for this patch" << endl;
                }
                //float cosAngleAtCapture;
                //closestExistingDist=std::min(closestExistingDist,distAtCapture);
            }


            //the first implementation only allows one texPatch for each patch.
            if(!createThisTexture){
                //cout << "we do not add a new texture since there already is one" << endl;
                patch->tex_patches_mutex.unlock();
                continue;
            }
        }else{
            createThisTexture = true;
        }
        patch->tex_patches_mutex.unlock();


        if(createThisTexture){

            int resX=bounds[i].width+1;
            int resY=bounds[i].height+1;

            shared_ptr<MeshTexture> meshTex =
                    mesh->genMeshTexture(MeshTexture::Type::COLOR8);
            /*
                    make_shared<MeshTexture>(nullptr,
                                             texAtlasRgb8Bit);
            */

            //meshTex->genGpuHandle();
            //set the pose at which this texture patch got captured
            meshTex->cam_pose = pose;
            //thats a good amount of
            int nrCoords =
                    patch->gpu.lock()->geom_tex->coords->getSize();

            shared_ptr<MeshTextureGpuHandle> meshTexGpu =
                    meshTex->genGpuResource(nrCoords,cv::Size2i(resX,resY));


            //create a task for new texture coordinates

            TexCoordGen::Task task;
            task.offset_x = bounds[i].x-0.5f;
            task.offset_y = bounds[i].y-0.5f;
            task.scale_x = 1.0f/(bounds[i].width + 1.0f);
            task.scale_y = 1.0f/(bounds[i].height + 1.0f);
            task.coords = meshTexGpu->coords->getStartingPtr();
            task.triangleCount = gpuPatch->triangles->getSize();

            task.triangles = gpuPatch->triangles->getStartingPtr();
            texGenTasks.push_back(task);

            //oh and also do this for all the double stitches
            for(shared_ptr<DoubleStitch> stitch : patch->double_stitches){
                if(stitch->patches[0].lock() != patch){
                    continue;
                }
                if(!stitch->isPartOfActiveSet(activeSet.get())){
                    stitch->isPartOfActiveSet(activeSet.get());
                    assert(0);//This actually should not happen
                    continue;
                }
                shared_ptr<TriangleBufConnector> gpuStitch = stitch->triangles_gpu.lock();
                if(gpuStitch == nullptr){
                    assert(0);
                    continue;
                }
                task.triangleCount = gpuStitch->getSize();
                task.triangles = gpuStitch->getStartingPtr();
                texGenTasks.push_back(task);
            }

            //also triple stitches
            for(shared_ptr<TripleStitch> stitch : patch->triple_stitches){
                if(stitch->patches[0].lock() != patch){
                    continue;
                }
                if(!stitch->isPartOfActiveSet(activeSet.get())){
                    assert(0); // again, i don't think this should happen
                    continue;
                }
                shared_ptr<TriangleBufConnector> gpuStitch =
                        stitch->triangles_gpu.lock();
                if(gpuStitch == nullptr){
                    assert(0);
                    continue;
                }
                task.triangleCount = gpuStitch->getSize();
                task.triangles = gpuStitch->getStartingPtr();
                texGenTasks.push_back(task);
            }

            //and also for copying the texture where it belongs to:
            CopyDescriptor copy;
            copy.x=bounds[i].x*_width;
            copy.y=bounds[i].y*_height;
            copy.width=bounds[i].width*_width;
            copy.height=bounds[i].height*_height;
            copy.targetWidth=resX;
            copy.targetHeight=resY;
            Rect2i outRect = meshTexGpu->tex->getRect();
            copy.targetX = outRect.x;
            copy.targetY = outRect.y;
            copy.output = meshTexGpu->tex->getCudaSurfaceObject();
            //meshTexGpu->sourceTex->getTex()->getCudaSurfaceObject();
            //newTexture->getSourceTexPatch()->getTex()->getCudaSurfaceObject();

            copies.push_back(copy);
            patchesWithColorUpdates.push_back(patch);


            //TODO: here!
            GpuCpuTexPair update;
            update.gpu = meshTexGpu;
            update.cpu = meshTex;
            updatedTextures.push_back(update);

            //TODO: this has to change when using multiple layers
            //cv::Mat debug(10,10,CV_32FC4);//somehow this does not invoke the same
            //cv::imshow("rgb data",debug);
            //cv::waitKey(1);
            //issues!
            meshTexturesBeingReplaced.push_back(gpuPatch->texs[0]);
            gpuPatch->texs[0] = meshTexGpu;
            patch->removeTexPatches(texPatchesToDelete);
            patch->addTexPatch(meshTex);
            meshTex->gpu = meshTexGpu;
            /*patch->texPatchesMutex.lock();
            patch->texPatches.push_back(meshTex);
            //+ delete the ones we want to delete!!
            patch->texPatchesMutex.unlock();
            */
        }
    }



    TexCoordGen::genTexCoords(texGenTasks,mvp,
                              mesh->m_gpuGeomStorage.patchInfoBuffer->getCudaPtr(),
                              mesh->m_gpuGeomStorage.vertexBuffer->getCudaPtr());

    copyToTinyPatches(rgbIn->getCudaTextureObject(),copies); //1.milliseconds for a full image (should be way less if the images are smaller)



    //updating the low detail map probably also requires a new version of the
    //header TODO: update headers on the fly and just set the headers to the new version in one (all blocking) kernel call.
    activeSet->reuploadHeaders();


    //also update the low detail map:
    mesh->lowDetailRenderer.updateColorForPatches(patchesWithColorUpdates);



    //TODO: download these textures!
    //cout << "[ScaleableMap::applyNewColorData] TODO: Trigger download of these "
    //        "textures" << endl;
    for(GpuCpuTexPair update : updatedTextures){
        //update.gpu->downloadToWhenFinished = update.cpu;
        update.gpu->gpu_data_changed = true;

        //TODO: delete if these proves to be useless
        /*
        update.cpu->textureMostCurrent = update.gpu->tex;
        update.cpu->texCoordsGpu = update.gpu->coords;
         */

    }

    return;
    //DEBUG MEASURES
/*
    cv::Mat rgbInput(480,640,CV_8UC4);
    rgbIn->downloadData(rgbInput.data);
    imshow("rgb Input1",rgbInput);

    for(size_t i=0;i<copies.size();i++){
        //continue;
        cout << "DEBUG: somehow we can't read from the input texture" << endl;
        if(copies[i].targetHeight < 40 || copies[i].targetWidth < 40 ){
            //continue;
        }
        ///TODO: find the bug causing this.
        //some serious issue at collecting the texture coordinates otherwise none of these images would appear much bigger than they are.
        cv::Mat test(1024,1024,CV_32FC4);

        visiblePatches[i]->texPatches[0]->getSourceTexPatch()->
                getTex()->downloadData(test.data);
        char name[100];
        sprintf(name,"patch (%d x %d)",copies[i].targetWidth,copies[i].targetHeight);
        cv::imshow("texture copy",test);
        cv::waitKey();
    }
    */


}


void Texturing::GenLookupTexGeom(ActiveSet *activeSet,
                                               std::vector<std::shared_ptr<MeshPatch>> &patches)
{
    vector<shared_ptr<MeshTexture>> textures;
    for(size_t i=0;i<patches.size();i++){
        textures.push_back(patches[i]->geom_tex_patch);
    }
    GenLookupTex(activeSet, patches, textures);
}



//TODO: maybe relocate this function into another class?
//also maybe directly operate on the patch
void Texturing::GenLookupTex(ActiveSet *activeSet,
                                           std::vector<std::shared_ptr<MeshPatch> > &patches,
                                           std::vector<std::shared_ptr<MeshTexture>> &textures,
                                           bool dilate){
    vector<DilationDescriptor> dilations;
    dilations.reserve(patches.size());
    meshReconstruction->m_informationRenderer.bindRenderTriangleReferenceProgram();

    for(size_t i = 0; i < patches.size();i++){
        std::shared_ptr<MeshPatch> patch = patches[i];
        std::shared_ptr<MeshTexture> texture = textures[i];
        shared_ptr<MeshTextureGpuHandle> gpuTex= texture->gpu.lock();
        if(gpuTex==nullptr){
            cout << "[ScaleableMap::generateLookupTexGeom] "
                    "There is no texture on the gpu" << endl;
            continue;
        }

        Rect2i r = texture->getLookupRect();
        //TODO: the scissor test probably is a little wasteful (a quad would be way better)
        //TODO: remove this, it is pulled into renderTriangleReferences
        /*
        Rect2i r = texture->getLookupRect();
        glBindFramebuffer(GL_FRAMEBUFFER, gpuTex->refTex->getFBO());

        gfx::GLUtils::checkOpenGLFramebufferStatus("ScaleableMap::finalizeGeomTexOfNewPatches");
        glEnable(GL_SCISSOR_TEST);
        glScissor(r.x,r.y,r.width,r.height);
        glViewport(r.x,r.y,r.width,r.height);
         */
        //to solve this we might want to draw a quad
        meshReconstruction->m_informationRenderer.renderTriangleReferencesForPatch(activeSet,
                                                               patches[i],
                                                               texture);
        //glFinish();//let the opengl stuff render before we download it.
        //how about imshowing the result


        //glDisable(GL_SCISSOR_TEST);



        if(dilate){
            DilationDescriptor dilation;
            dilation.target = gpuTex->ref_tex->getCudaSurfaceObject();
            dilation.width=r.width;
            dilation.height=r.height;
            dilation.x=r.x;
            dilation.y=r.y;
            dilations.push_back(dilation);
        }
    }

    glFinish();//let the opengl stuff render before we download it.

    //At last we dilate the lookup of
    if(dilate){
        dilateLookupTextures(dilations);
        cudaDeviceSynchronize();

    }
    for(size_t i=0;i<patches.size();i++){
        patches[i]->geom_tex_patch->ref_tex_filled = true;
    }


    auto end = chrono::high_resolution_clock::now();
}






