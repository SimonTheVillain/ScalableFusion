//
// Created by simon on 8/2/19.
//

#include "Labelling.h"
#include "meshReconstruction.h"
#include "cuda/coalescedMemoryTransfer.h"
#include "cuda/labelling.h"
#include "cuda/coalescedMemoryTransfer.h"
#include "gpu/ActiveSet.h"

#include "camera.h"


using namespace std;
using namespace cv;
using namespace Eigen;


typedef CoalescedGpuTransfer::SetTaskTemp<GpuTextureInfo> GpuTexInfoTask;

typedef CoalescedGpuTransfer::CpyTaskTemp<GpuTextureInfo> DEBUGTask;
void Labelling::ApplyLabels(std::shared_ptr<ActiveSet> activeSet, cv::Mat labels, Eigen::Matrix4f pose) {
    assert(0);
}

void Labelling::ProjectLabels(std::shared_ptr<ActiveSet> activeSet, cv::Mat &labels,
                              std::shared_ptr<gfx::GpuTex2D> dStdTex, Eigen::Matrix4f pose) {

    MeshReconstruction *mesh = meshReconstruction;
    //first we create a texture: gfx::GpuTex2D
    //internal formatGL_R32I;

    int width = labels.cols;
    int height = labels.rows;

    //GL_R
    std::shared_ptr<gfx::GpuTex2D> labelTex =
            std::make_shared<gfx::GpuTex2D>( mesh->garbageCollector,
                                             GL_R32I,GL_RED_INTEGER,GL_INT,
                                             width,height,
                                             true, nullptr,GL_NEAREST);
    labelTex->uploadData(labels.data);

    vector<shared_ptr<MeshTextureGpuHandle>> securedPatches;


    //for proper labelling we need 3 kinds of tasks:
    //copying over texture coordinates from the geometry texture to be able to
    // create the lookup textures
    std::vector<CoalescedGpuTransfer::TaskD2D> copyTasks;

    //creating lookup textures
    vector<shared_ptr<MeshPatch>> patchesUpdateLookup;
    vector<shared_ptr<MeshTexture>> texturesUpdateLookup;


    //and the actual labelling tasks:
    vector<gpu::Labelling::SegProjTask> labellingTasks;
    vector<gpu::Labelling::InitializeTask> initializationTasks;
    vector<shared_ptr<MeshTextureGpuHandle>> labellingTexturesDebug;

    vector<GpuTexInfoTask> infoUpdateTasks;
    vector<DEBUGTask> debugTasks;


    //the first two could actually be shared with the geometry texture.
    //TODO: do this at some point... make this info shareable



    //actually, we need to secure the labels in at least this active set.


    for(std::shared_ptr<MeshPatch> patchCpu :
            activeSet->retainedMeshPatchesCpu){

        patchCpu->labelTexPatchMutex.lock();
        shared_ptr<MeshPatchGpuHandle> patchGpu = patchCpu->gpu.lock();
        if(patchGpu == nullptr){
            continue;
        }

        if( patchCpu->labelTexPatch == nullptr ){
            //create a new texture, use the same resolution as the geometry
            //texture
            shared_ptr<MeshTextureGpuHandle> geomTexGpu =
                    patchCpu->geomTexPatch->gpu.lock();
            cv::Rect2i roi = geomTexGpu->tex->getRect();
            cv::Size2i size = roi.size();
            //int w = size.width;
            //int h = size.height;
            size_t texCoordCount = geomTexGpu->coords->getSize();

            cout << "[ScaleableMap::projectLabels] A lot more work needs to be "
                    "done here!" << endl;
            //shared_ptr<MeshTexture> texPatch
            shared_ptr<MeshTexture> meshTexture =
                    mesh->genMeshTexture(MeshTexture::Type::integerLabels);

            shared_ptr<MeshTextureGpuHandle> texGpuHandle =
                    meshTexture->genGpuResource(texCoordCount,
                                                size);



            meshTexture->gpu = texGpuHandle;//TODO: how to secure the gpuTexPatch?
            patchCpu->labelTexPatch = meshTexture;
            patchGpu->setLabelTex(texGpuHandle);


            //assert(0); //retain the texture in the gpu handle


            shared_ptr<MeshPatchGpuHandle> patchGpu = patchCpu->gpu.lock();


            if(patchGpu != nullptr){
                shared_ptr<MeshTextureGpuHandle> texPatchGpu = patchGpu->labelTex;

                //TODO: set the label to -1
                //also don't create a new texture every time!!!!!
                gpu::Labelling::InitializeTask initializeTask;
                initializeTask.destSurf = texPatchGpu->tex->getCudaSurfaceObject();
                initializeTask.destRect = texPatchGpu->tex->getRect();
                initializationTasks.push_back(initializeTask);

                patchesUpdateLookup.push_back(patchCpu);
                texturesUpdateLookup.push_back(meshTexture);

                CoalescedGpuTransfer::TaskD2D texCoordTask;
                texCoordTask.count = texCoordCount;
                texCoordTask.sourceIndex =
                        geomTexGpu->coords->getStartingIndex();
                texCoordTask.destinationIndex =
                        texPatchGpu->coords->getStartingIndex();


                copyTasks.push_back(texCoordTask);

                GpuTexInfoTask infoTask;
                infoTask.dst =
                        &(patchGpu->patchInfos->getStartingPtr()->
                                segmentationTexture);

                infoTask.value = texPatchGpu->genTexInfo();
                GpuTextureInfo info = texPatchGpu->genTexInfo();
                //infoTask.value = patchGpu->texs[0]->genSrcTexInfo(); //DEBUG
                //infoTask.value = patchGpu->geomTex->genSrcTexInfo();//another debug
                infoUpdateTasks.push_back(infoTask);


                DEBUGTask debugTask;

                debugTask.src =
                        &(patchGpu->patchInfos->getStartingPtr()->stdTexture);
                debugTask.dst =
                        &(patchGpu->patchInfos->getStartingPtr()->
                                segmentationTexture);
                debugTasks.push_back(debugTask);
            }else{
                assert(0);//this means we lost the newly created tex patch
            }
        }

        shared_ptr<MeshTexture> texPatch = patchCpu->labelTexPatch;
        shared_ptr<MeshTextureGpuHandle> gpuTexPatch = texPatch->gpu.lock();
        if(gpuTexPatch==nullptr){
            assert(0);
        }
        gpu::Labelling::SegProjTask task;
        //fill up task
        task.subchannel = static_cast<int>(0);
        task.destSurf =
                gpuTexPatch->tex->getCudaSurfaceObject();
        //wow! this is too long
        task.destination = gpuTexPatch->tex->getRect();
        //position (top left)
        task.lookup = gpuTexPatch->refTex->getRect().tl();
        task.lookupSurf = gpuTexPatch->refTex->getAtlasTex()->getTex2D()->getCudaSurfaceObject();


        task.vertexDestStartInd = patchGpu->verticesSource->getStartingIndex();
        labellingTasks.push_back(task);//append task

        labellingTexturesDebug.push_back(gpuTexPatch);
        patchCpu->labelTexPatchMutex.unlock();

    }

    //first we need to copy over the texture coordinates from geometry to the
    //label patch
    CoalescedGpuTransfer::device2DeviceSameBuf(mesh->m_gpuGeomStorage.texPosBuffer->getCudaPtr(),
                                               copyTasks);
    //now we do the lookup textures
    cout << "DEBUG: some of these textures are missing?, but why?" << endl;

    /*
    cv::Mat wait(1000,1000,CV_32FC4);
    cv::imshow("waitALITTLE",wait);
    cv::waitKey();
    */
    mesh->texturing.GenLookupTex(activeSet.get(),
                           patchesUpdateLookup,
                           texturesUpdateLookup,
                           false); // no dilation of lookup

    int minuseins = -1;
    Vector4f initialValue(*((float*) &minuseins),0,0,0);
    gpu::Labelling::initializeSurfaces<Vector4f>(initializationTasks, initialValue);
    cudaDeviceSynchronize(); //which it is anyway doing!!!!!!!!


    Matrix4f proj =
            Camera::genScaledProjMatrix(mesh->params.depthfxycxy,
                                        mesh->params.depthRes);
    proj = Camera::genProjMatrix(mesh->params.depthfxycxy);
    //cout << proj << endl;


    Matrix4f _pose = pose.inverse();
    Matrix4f proj_pose =  proj * _pose;
    //cout << "projection in the labelling part of things" << endl << proj << endl;
    //in the end do the labelling
    gpu::Labelling::labelSurfaces(labellingTasks,
                                  labelTex->getCudaSurfaceObject(),
                                  dStdTex->getCudaSurfaceObject(),
                                  cv::Size2i(width,height),
                                  _pose,
                                  proj_pose,
                                  mesh->m_gpuGeomStorage.vertexBuffer->getCudaPtr(),
                                  mesh->m_gpuGeomStorage.texPosBuffer->getCudaPtr(),
                                  mesh->m_gpuGeomStorage.triangleBuffer->getCudaPtr(),
                                  mesh->m_gpuGeomStorage.patchInfoBuffer->getCudaPtr());







    //TODO: update descriptor on gpu
    CoalescedGpuTransfer::upload(infoUpdateTasks);
    //CoalescedGpuTransfer::copy(debugTasks);//DEBUG!!!!


    //cv::waitKey(); // maybe this helps to retain all the newly generated textures
    return;//debug for everything below
    //do some debug output
    /*
    for(size_t i = 0; i< labellingTasks.size();i++){

        shared_ptr<MeshTextureGpuHandle> texPatchGpu =
                labellingTexturesDebug[i];

        //TODO: present to the user to check for errors!
        cv::Rect2i roi =
                texPatchGpu->sourceTex->getRectOfInterest();
        cv::Mat data(roi.height,roi.width,CV_32FC4);
        cout << roi << endl;//these resolutions are all suspicious
        texPatchGpu->sourceTex->downloadData(data.data);
        //data.setTo(cv::Scalar(100000,0,100000,0));
        for(int j=0;j< data.cols*data.rows;j++){
            cv::Vec4i vi = data.at<cv::Vec4i>(j);
            cv::Vec4f vf(0.0f,0.0f,0.0f,1.0f);
            if(vi[0] == -1){//blue... when it was ruled out by whatever
                vf = cv::Vec4f(1.0f,0.0f,0.0f,1.0f);
            }else if(vi[0] == -2){//the depth is nan at this pixel.... so this should not be -> red
                vf = cv::Vec4f(0.0f,0.0f,1.0f,1.0f);
            } else if(vi[0] == -3){//ruled out by depth test-> yellow?
                vf = cv::Vec4f(0.0f,1.0f,1.0f,1.0f);
            } else if(vi[0] == -4){//ruled out by being out of bounds -> white
                vf = cv::Vec4f(1.0f,1.0f,1.0f,1.0f);
            } else if(vi[0] == 100000){ //green: set to the anticipated value -> green
                vf = cv::Vec4f(0.0f,1.0f,0.0f,1.0f);
            }else{
                cout << vi[0] << endl;
            }


            data.at<cv::Vec4f>(j) = vf;
        }

        cv::imshow("patchwiseLabel",data);


        //also present the lookup texture
        roi = texPatchGpu->refTex->getRectOfInterest();//is the same anyway

        data = cv::Mat(roi.height,roi.width,CV_32FC4);

        texPatchGpu->refTex->downloadData(data.data);
        */
    /*
    castF16SurfaceToF32Buffer(
                texPatchGpu->refTex->getTex()->getCudaSurfaceObject(),
                roi.x,roi.y,
                roi.width,roi.height,
                reinterpret_cast<float*>(data.data),4);
                */

    /*
    cv::imshow("geometryLookup",data);


    cv::waitKey();
    //assert(0);
     */
    //}
}
