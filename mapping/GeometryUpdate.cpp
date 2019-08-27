//
// Created by simon on 8/2/19.
//

#include "GeometryUpdate.h"
#include "meshReconstruction.h"
#include "ActiveSet.h"
#include "camera.h"
#include "cuda/stdTexUpdate.h"
#include "utils/gpuNormSeg.h"
#include "graph/DeformationNode.h"

#include <iostream>
#include <unordered_map>

#include "cuda/xtionCameraModel.h"
#include "cuda/coalescedMemoryTransfer.h"

using namespace cv;
using namespace Eigen;
using namespace std;


void GeometryUpdate::Extend(std::shared_ptr<ActiveSet> activeSetOfFormerlyVisiblePatches,
                            std::shared_ptr<gfx::GpuTex2D> dStdTex,
                            cv::Mat& dStdMat,
                            Eigen::Matrix4f depthPoseIn,
                            std::shared_ptr<gfx::GpuTex2D> rgbTex,
                            Eigen::Matrix4f colorPoseIn){
    MeshReconstruction *mesh = meshReconstruction;
    //TODO: add everything from meshReconstructionLogic.cpp here!!!!!
    std::shared_ptr<ActiveSet> formerActiveSet = activeSetOfFormerlyVisiblePatches;

    auto start = std::chrono::system_clock::now();

    int width=dStdMat.cols;
    int height=dStdMat.rows;



    //prepare the intrinsic parameters of the depth cam
    float fxd = mesh->params.depthfxycxy[0];
    float fyd = mesh->params.depthfxycxy[1];
    float cxd = mesh->params.depthfxycxy[2];
    float cyd = mesh->params.depthfxycxy[3];

    Matrix4f projDepth = Camera::genProjMatrix(mesh->params.depthfxycxy);
    Matrix4f projDepthN =
            Camera::genScaledProjMatrix(mesh->params.depthfxycxy,
                                        mesh->params.depthRes);


    Matrix4f proj1Color = Camera::genProjMatrix(mesh->params.rgbfxycxy);//one to do what has to be done anyway


    //extract the visible patches from the active set
    vector<shared_ptr<MeshPatch>> visiblePatches =
            activeSetOfFormerlyVisiblePatches->retainedMeshPatchesCpu;
    //shared_ptr<ActiveSet> activeSetOfFormerlyVisiblePatches = set;

    //this might me more suitable for the beginning but lets do it here:
    mesh->m_informationRenderer.renderDepth(activeSetOfFormerlyVisiblePatches.get(),
                                      projDepthN,depthPoseIn);
    cv::Mat exGeom(height,width,CV_32FC4);//existing geometry
    mesh->m_informationRenderer.getDepthTexture()->downloadData(exGeom.data);
    //cv::imshow("projected existing geometry",exGeom);

    cv::Mat projDepthStd(height,width,CV_32FC4);

    mesh->m_informationRenderer.getStdTexture()->downloadData(projDepthStd.data);
    //cv::imshow("depth and standard deviations", dStdMat*0.1);
    //cv::imshow("projected existing depth and standard deviations",projDepthStd*0.1);
    //cv::waitKey(1);
    //cout << "color at certain pixel" << exGeom.at<Eigen::Vector4f>(10,10) << endl;//debug
    //cv::waitKey(); //debug at this point



    //TODO: DEBUG: //check all the points if they have still edges assigned to them:






    /*******************************************STITCHING NEW********/
    //vector<Edge*> border = createBorder(activeSetOfFormerlyVisiblePatches->retainedMeshPatchesCpu);
    //TODO: fill a list of borders with edges...(without having to )
    vector<vector<Edge>> borders;
    Matrix4f proj_pose = projDepth * depthPoseIn.inverse();
    int debugPatchCount = activeSetOfFormerlyVisiblePatches->retainedMeshPatchesCpu.size();

    //debugCheckTriangleNeighbourConsistency(activeSetOfFormerlyVisiblePatches->retainedMeshPatchesCpu);
    //debugCheckTriangleEdgesUnregistered(GetAllPatches());

    stitching.genBorderList(activeSetOfFormerlyVisiblePatches->retainedMeshPatchesCpu,borders,proj_pose);
    stitching.reloadBorderGeometry(borders);
    //TODO: reimplement this function to improve everything!!!
    stitching.rasterBorderGeometry(borders,depthPoseIn,projDepth,exGeom);







    //this is a premilary measure to get the geomatry adding running....
    float geometryAssignThreshold=0.05f;//every point within 5cm of existing geometry is considered part of that geometry


    /**
     * TODO: get rid of this code: It is done in the segmentation part anyway.
     * Altough the code for projecting points should be separate from this.
     *
     */
    //cv::Mat texPos(m_depth.cols,m_depth.rows,CV_32FC2);
    cv::Mat points(dStdMat.rows,dStdMat.cols,CV_32FC4);

    for(int i=0;i<dStdMat.rows;i++){
        for(int j=0;j<dStdMat.cols;j++){
            float z = dStdMat.at<cv::Vec4f>(i,j)[0];
            //float z=(1.0f/5000.0f)*(float)depthIn.at<uint16_t>(i,j);//these 5000.0f are coming from the TUM dataset
            points.at<Vector4f>(i,j)=Vector4f(((float)j-cxd)*z/fxd,((float)i-cyd)*z/fyd,z,1.0f);
            //texPos.at<Vector2f>(i,j)=Vector2f(float(j)/float(width),float(i)/float(height));
            if(isnan(z) || z==0.0f){// || z > m_maxDistance){
                //actually we want this to be done in the segmentation method
                points.at<Vector4f>(i,j)=Vector4f(NAN,NAN,NAN,NAN);
            }

            float exZ=exGeom.at<Vector4f>(i,j)[2];
            if(!isnan(exZ)){
                //Threshold dependant on the standard deviation
                float thresh = std::max(
                        projDepthStd.at<Vec4f>(i,j)[2], //standard deviation. TODO: WRONG DATA FORMAT!!!
                        dStdMat.at<Vec4f>(i,j)[2]);//something not working here
                thresh = xtionStdToThresholdSeg(thresh);
                if(z>exZ-thresh){//geometryAssignThreshold){
                    points.at<Vector4f>(i,j)=Vector4f(NAN,NAN,NAN,NAN);
                }
            }
        }
    }
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    cv::imshow("novel geometry",points);
    cv::waitKey();//this is a debug measure.... because everything is shit
#endif


    //cv::imshow("novel geometry",points);
    //cv::waitKey();//this is a debug measure.... because everything is shit
    cv::Mat meshPointers(height,width,CV_32SC2);//actually we store pointers in this


    mesh->gpuPreSeg->fxycxy = mesh->params.depthfxycxy;
    mesh->gpuPreSeg->maxDistance = mesh->getMaxDistance();
    //do a proper segmentation on all the pixel not part of the existing geometry
    mesh->gpuPreSeg->segment(dStdTex,projDepthStd,exGeom);

    Mat seg = mesh->gpuPreSeg->getSegmentation();
    int segCount = mesh->gpuPreSeg->getSegCount();//count of segments.



    //waitKey();//debug because nothin works

    //*****************************************TOOOOODOOOOOO*********************
    //TODO: put this piece of code into the meshIt part!!!!!
    vector<shared_ptr<MeshPatch>> newSharedMeshPatches;
    //the same as the one above but with shared elements
    for(int i=0;i<segCount;i++){
        std::shared_ptr<MeshPatch> meshPatch = mesh->genMeshPatch();
        newSharedMeshPatches.push_back(meshPatch);//storing all shared_ptr to the next mesh patch
    }
    for(int i=0;i<seg.rows;i++){
        for(int j=0;j<seg.cols;j++){
            int index=seg.at<int32_t>(i,j);
            //cout << index << endl;
            if(index!=-1){
                meshPointers.at<MeshPatch*>(i,j)=newSharedMeshPatches[index].get();
            }else{
                meshPointers.at<MeshPatch*>(i,j)=nullptr;

            }
        }
    }



    //cv::imshow("segmentation of new input",generateColorCodedTexture(meshPointers));
    //cv::waitKey();

#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    cv::imshow("segmentation of new input",generateColorCodedTexture(meshPointers));
    cv::waitKey();
#endif


    //add vertices to the accroding patches and store indices into a Mat

    //After we have all the necessary points within each of the patches we can use the
    //them to calculate the center points, bounding sphere and principal point.


    auto start1 = std::chrono::system_clock::now();
    //mesh that stuffcv:

    cv::Mat vertexIndices(height,width,CV_32SC1);
    vertexIndices.setTo(cv::Scalar(-1));//TODO: remove this line, should not be necessary

    meshing.MeshIt(points,meshPointers,vertexIndices,dStdMat,mesh->params.maxDepthStep,depthPoseIn);
    //mesh->debugCheckTriangleNeighbourConsistency(mesh->GetAllPatches());//TODO: remove this debug afterwards


    for(size_t i = 0;i<newSharedMeshPatches.size();i++){
        //TODO: unify these functions and maybe do this at the very end of everything!!!!!
        newSharedMeshPatches[i]->updatePrincipalPlaneAndCenter();
        newSharedMeshPatches[i]->updateSphereRadius();
    }




    auto end1 = std::chrono::system_clock::now();
    auto elapsed1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    //present
    std::cout << "[GeometryUpdate::Extend] time consumed by meshing: " << elapsed1.count() << "ms" << std::endl;


    start1 = std::chrono::system_clock::now();




    /*********************************Initial Stitching!! NEW***************************/


    vector<weak_ptr<GeometryBase>> stitchList;



    //TODO: reinsert code and fix bugs
    //debugCheckTriangleNeighbourConsistency(GetAllPatches());
    stitching.stitchOnBorders(borders,
                    depthPoseIn,projDepth,
                    projDepthStd,exGeom,
                    points,dStdMat,
                    mesh->generateColorCodedTexture(meshPointers),
                    meshPointers,vertexIndices,stitchList);


    stitching.freeBorderList(borders);




    /******************************************Initial Stitching!! NEW***********************/





    end1 = std::chrono::system_clock::now();
    elapsed1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    //present
    std::cout << "[GeometryUpdate::Extend] time consumed by stitching: " << elapsed1.count() << "ms" << std::endl;
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    cv::imshow("project old standard deviation texture to new",exColor);
#endif

    start1 = end1;




    //remove patches with zero triangles. Even when they pose as a source for stitches
    /*
    unordered_map<GeometryBase*,weak_ptr<GeometryBase>> hateThis;

    for(int i=0;i<stitchList.size();i++){
        hateThis[stitchList[i].lock().get()]=stitchList[i];
    }
     */

    vector<shared_ptr<MeshPatch>> listOfMeshPatchesToKeep;
    vector<shared_ptr<MeshPatch>> setOfPatchesToBeRemoved;

    for(size_t i=0;i<newSharedMeshPatches.size();i++){
        shared_ptr<MeshPatch> &patch = newSharedMeshPatches[i];
        if(patch->triangles.size()==0){
            //cout << "here everything goes to shit " << endl;
            //if the patch does not have triangles we go over all stitches and their respective triangles and delete them
            //remove
            int debugDoubleStitchesBefore = patch->doubleStitches.size();
            for(size_t j = 0;j<patch->doubleStitches.size();j++){
                shared_ptr<DoubleStitch> stitch = patch->doubleStitches[j];
                int debugCountBefore = patch->doubleStitches.size();
                stitch->removeFromPatches(patch);//this should always remove one reference from this patch
                if(debugCountBefore != patch->doubleStitches.size()){
                    //assert(0);
                }
                stitch->deregisterTriangles();
            }
            if(patch->doubleStitches.size()>0){
                //assert(0);//this is weired!!!!!!! didn't we delete all double stitches?
            }

            for(size_t j=0;j<patch->tripleStitches.size();j++){
                shared_ptr<TripleStitch> stitch = patch->tripleStitches[j];
                stitch->removeFromPatches(patch);
                stitch->deregisterTriangles();
            }
            if(patch->doubleStitches.size()>0){
                //assert(0);//this is weired!!!!!!! didn't we delete all double stitches?
            }
            setOfPatchesToBeRemoved.push_back(patch);
        }else{
            //if the patch has real triangles we keep it
            listOfMeshPatchesToKeep.push_back(patch);
        }
    }

    newSharedMeshPatches = listOfMeshPatchesToKeep;

    for(size_t i=0;i<setOfPatchesToBeRemoved.size();i++){
        //Some of the patches don't have any connection or triangle at all
        //and therefore they would not get deleted until now.
        //Any unconnected or deleted but empty (of triangle) patch
        //gets deleted now.
        mesh->removePatch(setOfPatchesToBeRemoved[i]);
    }
    setOfPatchesToBeRemoved.clear();


    /*
    vector<shared_ptr<MeshPatch>> allPatches = mesh->GetAllPatches();
    for(int i=0;i<allPatches.size();i++){
        for(int k=0;k<allPatches[i]->triangles.size();k++){
            for(int l=0;l<3;l++){
                if(allPatches[i]->triangles[k].neighbours[l].debug &&
                   allPatches[i]->triangles[k].neighbours[l].valid()){
                    assert(0);
                    //TODO: remove this! I don't think this is necessary anymore!
                    //hate this contains all stitches created in the inter frame stitching code.....
                    // we should only compare with deleted ones.!?
                    //assert(hateThis.count(allPatches[i]->triangles[k].neighbours[l].ref.container)==1);

                    //there is a triangle pointing to a stitch triangle even though its stitch does not exist anymore
                }
                if(hateThis.count(allPatches[i]->triangles[k].neighbours[l].ref.container) != 0){
                    //assert(hateThis[allPatches[i]->triangles[k].neighbours[l].ref.container].use_count()>0);
                }
            }
        }
    }
    */


    end1 = std::chrono::system_clock::now();
    elapsed1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    //present
    std::cout << "[GeometryUpdate::Extend] time consumed by cleaning up the geometry " << elapsed1.count() << "ms" << std::endl;

    start1 = end1;



    start1 = end1;
    meshing.GenTexIndices(newSharedMeshPatches);


    //since the texture indices are set we can upload and create a new active set
    //most of the code below this active set creation can be put into the active set
    //creation routine

    visiblePatches.insert(visiblePatches.end(),
                          newSharedMeshPatches.begin(),
                          newSharedMeshPatches.end());
    shared_ptr<ActiveSet> newActiveSet =
            mesh->m_gpuGeomStorage.makeActiveSet(visiblePatches,mesh,
                                           true); // most of the members are initial
    newActiveSet->name = "createdInApplyNewData";


    //debugCheckTriangleNeighbourConsistency(visiblePatches);
    /*************************************TOOOODOOOOO************************/

    end1 = std::chrono::system_clock::now();
    elapsed1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    //present
    std::cout << "[GeometryUpdate::Extend] Time consumed loading the active set: " << elapsed1.count() << "ms" << std::endl;


    start1 = end1;





    newActiveSet->reuploadHeaders();
    mesh->texturing.GenerateGeomTex( newSharedMeshPatches,
                                    depthPoseIn,projDepth,
                                    dStdTex,
                                    newActiveSet);
    for(shared_ptr<MeshPatch> patch : newSharedMeshPatches){
        //TODO: test patch
        if(!patch->isPartOfActiveSetWithNeighbours(newActiveSet.get())){
            assert(0);//all of the new ones should be loaded
        }
        if(patch->geomTexPatch->gpu.lock() ==nullptr){
            assert(0);//whyyyyy. the geometry textures should be secured at
            //this point
        }
    }

    //until here..... not further (not further we implemented stuff)
    //(and with we i mean me)



    end1 = std::chrono::system_clock::now();
    elapsed1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    //present
    std::cout << "[GeometryUpdate::Extend] time consumed by finalizing the geom tex Coordinates of new patches: " << elapsed1.count() << "ms" << std::endl;



    start1 = end1;


    //add and fill new color patches to the surface
    mesh->texturing.ApplyColorData(newSharedMeshPatches,
                      rgbTex,colorPoseIn,proj1Color,
                      newActiveSet);

    //after doing the textures and geometry we are supposed to be done with this
    //active set
    newActiveSet->reuploadHeaders();

    //cv::waitKey();
    //the apply new color data is supposed to create new texture coordinates as well as creating new textures
    //this and the above function should replace the two steps below




    end1 = std::chrono::system_clock::now();
    elapsed1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    //present
    std::cout << "[GeometryUpdate::Extend] time consumed by applying new color data: " << elapsed1.count() << "ms" << std::endl;


    start1 = end1;



    //We want to create a new active set that replaces the old one.
    //The pointer to the active set should be secured by a mutex.
    mesh->m_gpuGeomStorage.deleteDebugTexReference = rgbTex->getGlHandle();




    //now that we have the geometry on the cpu now do the texture for the geometrical textures:


    end1 = std::chrono::system_clock::now();
    elapsed1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    //present
    std::cout << "[GeometryUpdate::Extend] Time consumed by "
                 "finalizing the new geom Tex patches: "
                 "" << elapsed1.count() << "ms" << std::endl;


    start1 = end1;


    //before putting the mesh patches into the octree we want to
    //set the center points and the radii of the patches right
    for(size_t i=0;i<newSharedMeshPatches.size();i++){
        newSharedMeshPatches[i]->updateCenterPoint();
        newSharedMeshPatches[i]->updateSphereRadius();
    }

    cout << "after changing the center points and sphere radii" << endl;
    //add the objects to the low detail renderer
    auto startLowDetail = std::chrono::system_clock::now();
    mesh->lowDetailRenderer.addPatches(newSharedMeshPatches,-depthPoseIn.block<3,1>(0,3));
    auto endLowDetail = std::chrono::system_clock::now();

    std::cout << "[GeometryUpdate::Extend] Time consumed by generating the low detail update"
                 ": " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(
                      endLowDetail - startLowDetail).count() <<
              "ms" << std::endl;


    //at last we can add the newly created and filed mesh patches to the octree


    end1 = std::chrono::system_clock::now();
    elapsed1 =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    //present
    std::cout << "[GeometryUpdate::Extend] Time consumed by projecting the geometry to the geometry texture "
                 ": " << elapsed1.count() << "ms" << std::endl;

    mesh->octree.addObjects(newSharedMeshPatches);



    //TODO: remove
    //check if the newly created active set is completely loaded
    newActiveSet->checkForCompleteGeometry();


    mesh->activeSetExpand = newActiveSet;
    //this is to get rid of that stupid bug
    //debugRetainEveryActiveSet.push_back(newActiveSet);


    //debug... check if the active set has all the geometry textures
    for(shared_ptr<MeshPatch> patch : newActiveSet->retainedMeshPatchesCpu){
        //TODO: test patch
        if(!patch->isPartOfActiveSetWithNeighbours(newActiveSet.get())){
            continue;
        }
        if(patch->geomTexPatch->gpu.lock() ==nullptr){
            cout << "DEBUG/TODO: reinsert the assert at this point" << endl;

            //assert(0);//whyyyyy. the geometry textures should be secured at
            //this point
        }
    }




    end1 = std::chrono::system_clock::now();
    auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(end1-start);
    std::cout << "[GeometryUpdate::Extend] Time consumed for adding new "
                 "(novel) data took alltogether " << elapsed.count() << "ms" << endl;
    //warning: this are old values... i don't even know if they apply to
    //on the initial frame it takes 1.2sec
    //on every additional frame its about 300ms. Almost half of it is the
    //creation and upload of the new active set. (which is shit btw.)

    /*****************************UPDATING GRAPH***************************/
    vector<DeformationNode::NodePixPos> nodes(visiblePatches.size());
    for(size_t i=0;i<visiblePatches.size();i++){
        shared_ptr<MeshPatch> patch = visiblePatches[i];
        Vector3f pos = patch->getPos();
        Vector4f pos4(pos[0],pos[1],pos[2],1.0f);
        pos4 = proj_pose * pos4;
        pos4 *= 1.0f/pos4[3];
        nodes[i].node = patch->deformationNode;
        nodes[i].pixPos = cv::Vec2f(pos4[0],pos4[1]);
        nodes[i].pos = pos;

    }
    for(auto &node : nodes){
        node.node->findNeighbours(node.pos,node.pixPos,nodes);
    }
    /*********************************************************************************/




    /*
    //debug: There once was a memory leak
    cout << "[GeometryUpdate::Extend] "
               "The amount of patches on the rgb textures:"
         << this->texAtlasRgb8Bit->countPatches() << endl;


    cout << "[GeometryUpdate::Extend] "
               "The amount of patches on the geom lookup:"
         << this->texAtlasStdsLookup->countPatches() << endl;


    cout << "[GeometryUpdate::Extend] "
               "The amount of patches on the std tex1:"
         << this->texAtlasStds[0]->countPatches() << endl;


    cout << "[GeometryUpdate::Extend] "
               "The amount of patches on the std tex2:"
         << this->texAtlasStds[1]->countPatches() << endl;


    cout << "[GeometryUpdate::Extend] "
               "visible patches:"
         << newActiveSet->retainedMeshPatches.size() << endl;

    cout << "[GeometryUpdate::Extend] "
               "overall patches:"
         << m_patches.size() << endl;
    cout << "overall count of textures: " <<
            gfx::GpuTex2D::getTexCount() << endl;
    */

    //cv::waitKey(1);
    //cv::waitKey();//DEBUG
}

void GeometryUpdate::Update(std::shared_ptr<gfx::GpuTex2D> dStdTex,
            Eigen::Matrix4f depthPoseIn,
            std::shared_ptr<ActiveSet> &activeSet){
    MeshReconstruction *mesh = meshReconstruction;
    //return;//debug
    if(activeSet==nullptr){
        return;
    }
    //cout << "ScaleableMap::vertGeomTexUpdate DEBUG: this is not implemented yet" << endl;
    //return;

    auto start = chrono::high_resolution_clock::now();

    int width=dStdTex->getWidth();
    int height=dStdTex->getHeight();


    /**
     * weighting scheme:
     * o..... offset from triangulated surface
     * sm.... minimal possible (due to quantisation) standard deviation alike value
     * s..... standard deviation alike value
     * s'=s-sm the actual standard deviation we will be working on.
     * we want s to always be bigger than sm after an update
     * smk =min(smk-1, sm)
     * s'k = (s'k-1 * s')/(s'k-1 + s') the new standard deviation
     *
     * the update of the old offset is still done on the old wieghts.
     * ok=(o/s + ok-1/sk-1)/( (s * sk-1)/(s + sk-1))
     */


    Matrix4f proj = Camera::genProjMatrix(mesh->params.depthfxycxy);//one to do what has to be done anyway

    Matrix4f pose = depthPoseIn;
    Matrix4f _pose = pose.inverse();
    Matrix4f proj_pose= proj*_pose;

    //Get the position of the depth camera
    Vector4f camPos =
            Camera::calcCamPosFromExtrinsic(_pose);



    vector<shared_ptr<MeshPatch>> patches =
            activeSet->retainedMeshPatchesCpu;


    //creation of the descriptors for this job
    vector<gpu::UpdateDescriptor> descriptors;
    vector<shared_ptr<MeshPatch>> updatedPatches;

    //vector<shared_ptr<MeshTextureGpuHandle>> destTextures;
    vector<shared_ptr<TexAtlasPatch>> destTexHandles;//retain the destination tex handles for this function

    for(size_t i = 0; i< patches.size();i++){

        //TODO: maybe we should not update every visible patch. we might want
        //to check if it really is feasible on every patch. some of them
        //might be hidden or some of them might be too far away and the surface
        //estimate is already waaay better than what the sensor could deliver.
        //?????
        //TODO: maybe the generation of this descriptor fits into
        //the mesh patch class.
        shared_ptr<MeshPatch> patch = patches[i];

        shared_ptr<MeshPatchGpuHandle> gpu = patch->gpu.lock();
        if( !patch->isPartOfActiveSetWithNeighbours( activeSet.get() ) ){
            continue;
        }


        shared_ptr<MeshTextureGpuHandle> geomTexGpuHandle = patch->geomTexPatch->gpu.lock();
        if(geomTexGpuHandle == nullptr){
            MeshTextureGpuHandle *debugTex =
                    activeSet->retainedMeshPatches[i]->geomTex.get();
            cout << "how come this is not initialized yet?"<< endl;
            assert(0);
            continue;//actually this can happen in multithreaded mode
            assert(0);
        }
        if( geomTexGpuHandle->tex == nullptr ){
            continue;
        }




        if(!geomTexGpuHandle->checkRefTexDependencies()){
            //assert(0) because we might want to fix the dependencies before entering this loop
            //maybe even before entering this method because the refTexCould be fixed during
            //creation of the activeSet
            assert(0);//should basically do the same as the tests after
        }
        bool expired=false;
        //TODO: Shorten this by putting it into a method of the geometryBase class
        if(geomTexGpuHandle->refTexDependencies.size()==0){
            assert(0);
            continue;//don't do a update since the refTex obviously is not set (indicated by it having no dependencies)
            //TODO: create/update the refTex if all the necessary neighbours are also part of this set


        }
        for(size_t j = 0;j< geomTexGpuHandle->refTexDependencies.size();j++){
            MeshTextureGpuHandle::Dependency dependency = geomTexGpuHandle->refTexDependencies[j];
            //int currentGpuPosition =
            shared_ptr<GeometryBase> dependenceGeom = dependency.geometry.lock();
            if(dependenceGeom == nullptr ||
               dependenceGeom->getMostCurrentGpuTriangles() == nullptr){
                assert(0);
                //this should not happen at any time!!!!!!!!!
            }
            //shared_ptr<MeshPatchGpuHandle> dependencyPatchGpu = dependencyPatch->gpu.lock();
            //if(dependencyPatchGpu == nullptr){
            //    assert(0);
            //}
            if(dependenceGeom->trianglesVersion != dependency.trianglesVersion){
                expired = true;
                assert(0);
            }
            shared_ptr<TriangleBufConnector> dependenceTris = dependenceGeom->getMostCurrentGpuTriangles();
            int debug = dependenceTris->getStartingIndex();
            if(dependenceTris->getStartingIndex() != dependency.trianglePositionOnGpu){
                expired = true;
                assert(0);
            }
            //dependency.patch
        }
        if(expired){
            //TODO: rebuild the reference texture. (which is supposed to be done in the ActiveSet creation process!)
            assert(0);
        }

        //check if the patch is ready to be used
        /*
        if(!patch->isGeometryFullyAllocated()){ //maybe ask the same for the neighbours
            continue;
        }
         */

        //not run this if the cpu has content that is ahead of the gpu content
        //propably because it is not uploaded
        /*if(patch->geomTexPatch->cpuContentAhead){
            continue;
        }*/
        //also not run this if the reference texture is not filled
        if(!patch->geomTexPatch->refTexFilled){
            assert(0);
            continue;
        }
        updatedPatches.push_back(patch);

        gpu::UpdateDescriptor desc;
        shared_ptr<gfx::GpuTex2D> tex = geomTexGpuHandle->tex->getTex();
        //patch->geomTexPatch->getSourceTexPatch()->getTex();
        cv::Rect2i rect = geomTexGpuHandle->tex->getRect();
        float width = tex->getWidth();
        float height = tex->getHeight();
        desc.sourceGeometry = tex->getCudaSurfaceObject();

        //wow, this really has to be streamlined
        desc.source=rect;
        desc.sourceN = Rect2f(float(rect.x)/width,
                              float(rect.y)/height,
                              float(rect.width)/width,
                              float(rect.height)/height);
        desc.sourceSize = Size2i(width,height);


        //desc.patchInfoSlot = patch->gpuPatchInfoBuf.lock()->getStartingIndex();
        desc.patchInfoSlot = gpu->patchInfos->getStartingIndex();

        //set the references to the vertex data.
        desc.vertexSourceStartInd = gpu->verticesSource->getStartingIndex();

        desc.vertexCount = gpu->verticesSource->getSize();
        desc.vertexDestinationStartInd =
                gpu->verticesDest->getStartingIndex();

        desc.triangleSlot = gpu->triangles->getStartingIndex();
        desc.triangleCount = gpu->triangles->getSize();


        //now go for the destRect textures
        //shared_ptr<MeshTextureGpuHandle> destTex = make_shared<MeshTextureGpuHandle>();
        shared_ptr<TexAtlasPatch> destTex = mesh->texAtlasStds->getTexAtlasPatch(rect.size());
        //destTex->refTex = geomTexGpuHandle->refTex;
        //assert(0); // TODO: also copy over the  dependencies belonging to the referenceTexture if we are really copying

        tex = destTex->getTex();
        //OKOK,

        rect = destTex->getRect();
        //width and height of the atlas texture
        width = tex->getWidth();
        height = tex->getHeight();

        desc.destinationGeometry = tex->getCudaSurfaceObject();
        desc.destination = rect;
        desc.destinationN = Rect2f(float(rect.x)/width,
                                   float(rect.y)/height,
                                   float(rect.width)/width,
                                   float(rect.height)/height);
        desc.destinationSize = Size2i(width,height);


        desc.destinationReferences =
                geomTexGpuHandle->refTex->getCudaSurfaceObject();


        rect = geomTexGpuHandle->refTex->getRect();
        desc.referenceOffset = rect.tl();

        descriptors.push_back(desc);
        destTexHandles.push_back(destTex);


        /*
        //some more debugging efforts
        cv::Mat debug(rect.height,rect.width,CV_32FC4);
        geomTexGpuHandle->refTex->downloadData(debug.data);
        debugMats.push_back(debug);

        Rect2i r = geomTexGpuHandle->tex->getRect();
        //simple debug output
        cv::Mat test(r.height,r.width,CV_32FC4);

        cudaSurfaceObject_t surface =
                                    geomTexGpuHandle->tex->getCudaSurfaceObject();
        castF16SurfaceToF32Buffer(surface,
                                  r.x,r.y,
                                  r.width,r.height,
                                  (float*)test.data,
                                  4);
        debugMats2.push_back(test);

        r=destTex->getRect();
        surface =
                destTex->getCudaSurfaceObject();
        castF16SurfaceToF32Buffer(surface,
                                  r.x,r.y,
                                  r.width,r.height,
                                  (float*)test.data,
                                  4);
        debugMats3.push_back(test);
         */


    }



    //cout << "[ScaleableMap::vertGeomTexUpdate] after creating a the update descriptors" << endl;
    //waitKey();

    //make the according buffers resident on the gpu

    auto start1 = chrono::high_resolution_clock::now();

    if(destTexHandles.size() != updatedPatches.size()){
        assert(0);
    }


    //desperate debug measure
    for(std::shared_ptr<MeshPatch> patch : patches) {
        std::shared_ptr<MeshPatchGpuHandle> gpuPatch = patch->gpu.lock();
        if (gpuPatch == nullptr) {
            assert(0);
        }
        if(gpuPatch->geomTex == nullptr){
            assert(0);
        }
        if(gpuPatch->geomTex->tex == nullptr){
            assert(0);
        }
        if(gpuPatch->geomTex->refTex == nullptr){
            assert(0);
        }

    }


    int debug = updateGeometry(dStdTex->getCudaSurfaceObject(),
                               width,height,
                               descriptors,
                               camPos,
                               _pose,
                               proj_pose,
                               (GpuVertex*)mesh->m_gpuGeomStorage.vertexBuffer->getCudaPtr(),
                               (Vector2f*)mesh->m_gpuGeomStorage.texPosBuffer->getCudaPtr(),
                               (GpuTriangle*)mesh->m_gpuGeomStorage.triangleBuffer->getCudaPtr(),
                               (GpuPatchInfo*)mesh->m_gpuGeomStorage.patchInfoBuffer->getCudaPtr());

    /*
    if(debug!=-1){
        cout << "lets check whats wrong with this data structure" << endl;
        shared_ptr<MeshPatch> patch = updatedPatches[debug];
        std::shared_ptr<MeshPatchGpuHandle> gpuPatch = patch->gpu.lock();
        cv::Rect2i r = gpuPatch->geomTex->refTex->getRect();
        cv::imshow("refTex",debugMats[debug]);
        cv::imshow("srcTex",debugMats2[debug]*10.0f);
        cv::imshow("dstTex",debugMats3[debug]*10.0f);
        cv::waitKey();
        //indeed destination references and geometry seem to be on the same texture!!!!!! how is this even possible? this is different
        auto descriptor = descriptors[debug];


        assert(0);
    }
     */
    //cout << "here we should first do the vertex update and then the std texture update" << endl;

    /*
    cv::Mat debug(480,640,CV_32FC4);
    dStdTex->downloadData(debug.data);
    cv::imshow("debug",debug);
    cv::waitKey();
    */

    gpu::GeometryUpdate::calcCenterAndRadius(patches);

    cudaDeviceSynchronize();
    //unmap the graphics ressources.
    auto end1 = chrono::high_resolution_clock::now();
    /*std::cout << "[ScaleableMap::vertGeomTexUpdate] "
                 "Updating the geometry and geometry texture. Took GPU only: " <<
                 chrono::duration_cast<chrono::nanoseconds>(end1-start1).count() << "ns\n";*/



    //after updating the target we want to switch target and source
    for(size_t i=0;i<updatedPatches.size();i++){
        shared_ptr<MeshPatch> patch = updatedPatches[i];
        shared_ptr<MeshPatchGpuHandle> gpu = patch->gpu.lock();
        shared_ptr<MeshTextureGpuHandle> gpuTexPatch =
                patch->geomTexPatch->gpu.lock();


        gpu->geomTex->tex = destTexHandles[i];//swap the std texture
        gpu->swapSrcDst();//swap the vertices
        //TODO: get this swap to gpu header


        //TODO: get rid of all the rest!?

        //we trigger the reupload(update) of the patch header on the gpu
        patch->cpuTexPatchAhead = true;//maybe we want to do this update with a separate kernel call


        //TODO: get this swap to gpu header
        //TODO: and instead of swapping
        //gpu->swapSrcDst();
        //gpu->geomTex->swapSrcDst(); // forgot to swap the textures
        //patch->swapVertexTargetSource();
        patch->cpuInfoAhead = true;//maybe we want to do this update with a separate kernel call

        //trigger download of geometry in this case.
        gpu->gpuVerticesChanged = true;
        gpu->downloadToWhenFinished = patch;

        //mask the newly updated geom texture as something that should be downloaded
        //patch->geomTexPatch->gpuContentAhead = true;



        //enable the download of mesh textures
        //gpuTexPatch->downloadToWhenFinished = patch->geomTexPatch;
        gpuTexPatch->gpuDataChanged = true;




    }



    meshReconstruction->cleanupGlStoragesThisThread();
    //fboStorage.cleanupThisThread();

    //cout << "[ScaleableMap::vertGeomTexUpdate] almost at exit of this function" << endl;
    //waitKey();
}