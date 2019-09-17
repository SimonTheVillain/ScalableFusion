#include "lowDetailMapRenderer.h"
#include "meshStructure.h"
#include "gpuGeomStorage.h"

#include <iostream>
#include <memory>


#include <glUtils.h>

#include "gpuErrchk.h"
#include "coarseUpdate.h"
#include "../gpu/ActiveSet.h"

const std::string lowDetail_frag =
#include "lowDetail.frag"
;

const std::string lowDetail_vert =
#include "lowDetail.vert"
;

const std::string lowDetail_geom =
#include "lowDetail.geom"
;


const std::string lowDetailRefDepth_frag =
#include "lowDetailRefDepth.frag"
;

const std::string lowDetailRefDepth_vert =
#include "lowDetailRefDepth.vert"
;

const std::string lowDetailRefDepth_geom =
#include "lowDetailRefDepth.geom"
;



//the purpose of these should be obvious!!!!!
const std::string debug_frag =
#include "debug.frag"
;
const std::string debug_vert =
#include "debug.vert"
;




using namespace gfx;
using namespace std;
using namespace Eigen;

std::weak_ptr<gfx::GLSLProgram> LowDetailRenderer::s_shader;
std::weak_ptr<gfx::GLSLProgram> LowDetailRenderer::s_geometryShader;

LowDetailRenderer::LowDetailRenderer()
{


}

LowDetailRenderer::~LowDetailRenderer()
{

}

void LowDetailRenderer::initInGlContext()
{
    if(LowDetailRenderer::s_shader.use_count()){
        shader = LowDetailRenderer::s_shader.lock();
    }else{
        shader = make_shared<GLSLProgram>();
        shader->compileShader(lowDetail_frag,
                              GLSLShader::GLSLShaderType::FRAGMENT,
                              "lowDetail.frag");
        shader->compileShader(lowDetail_geom,
                              GLSLShader::GLSLShaderType::GEOMETRY,
                              "lowDetail.geom");
        shader->compileShader(lowDetail_vert,
                              GLSLShader::GLSLShaderType::VERTEX,
                              "lowDetail.vert");
        shader->link();

        s_shader=shader;



        debugShader = make_shared<GLSLProgram>();

        debugShader->compileShader(debug_frag,
                              GLSLShader::GLSLShaderType::FRAGMENT,
                              "debug.frag");
        debugShader->compileShader(debug_vert,
                              GLSLShader::GLSLShaderType::VERTEX,
                              "debug.vert");
        debugShader->link();

    }

    if(LowDetailRenderer::s_geometryShader.use_count()){
        geometryShader = LowDetailRenderer::s_geometryShader.lock();
    }else{
        geometryShader = make_shared<GLSLProgram>();
        geometryShader->compileShader(lowDetailRefDepth_frag,
                              GLSLShader::GLSLShaderType::FRAGMENT,
                              "lowDetailRefDepth.frag");
        geometryShader->compileShader(lowDetailRefDepth_geom,
                              GLSLShader::GLSLShaderType::GEOMETRY,
                              "lowDetailRefDepth.geom");
        geometryShader->compileShader(lowDetailRefDepth_vert,
                              GLSLShader::GLSLShaderType::VERTEX,
                              "lowDetailRefDepth.vert");
        geometryShader->link();

        s_geometryShader = geometryShader;

    }


}

void LowDetailRenderer::addPatches(std::vector<std::shared_ptr<MeshPatch> > &patchesIn,Eigen::Vector3f camPos)
{
    if(patchesIn.size()==0){
        return;
    }
    //prevent the render thread from swapping the buffers
    newBuffers=false;

    vector<weak_ptr<MeshPatch>> newPatches;
    vector<weak_ptr<CoarseTriangle>> newCoarseTriangles;



    for(size_t i = 0;i<patchesIn.size();i++){
        MeshPatch &patch = *(patchesIn[i].get());

        if(patch.id == 620){
            //cout << " halt stop, jetzt debugge ich " << endl;
        }
        if(patch.vertices.size()==1){
            cout << "this definitely should not be, "
                    "patches with single vertices (or zero triangles)" << endl;
            //assert(0);
        }

        //add this patch to our array
        int index = patches.size()+newPatches.size();
        newPatches.push_back(patchesIn[i]);
        patch.indexWithinCoarse = index;
        //iterate neighbours
        set<shared_ptr<MeshPatch>> neighbours = patch.getNeighbours();
        /*set<shared_ptr<MeshPatch>> neighboursDebug = patch.getNeighboursDebug();
        if(neighbours.size()!=neighboursDebug.size()){
            cout << "DEBUG: here we might get the divergence when creating the "
                    "low detail map. maybe we check if the triple stitches are implemented right" << endl;
            cout << "propably they are though... lets just check" << endl;
            //assert(0);
        }*/
        //lets check triple stitches.... if they are consistent with the remaining data structure.
        patch.triple_stitch_mutex.lock();
        for(size_t j=0;j<patch.triple_stitches.size();j++){
            shared_ptr<TripleStitch> tripleStitch = patch.triple_stitches[j];
            for(size_t k=0;k<3;k++){
                bool found=false;
                for(size_t l=0;l<tripleStitch->patches[k].lock()->triple_stitches.size();l++){
                    if(tripleStitch->patches[k].lock()->triple_stitches[l] ==
                            tripleStitch){
                        found=true;
                    }
                }
                if(found==false){
                    cout << "DEBUG: obviously the structure is fragmented" << endl;
                    assert(0);
                }
            }
        }
        patch.triple_stitch_mutex.unlock();


        //This is a dirty workaround for something that should be doable in the next loop
        //patch.tripleStitchMutex.lock();
        //to preventing lock at triangle creation we do not lock here
        //(even if we should)
        //so, even tough we do this here, why isn't it working????????
        for(size_t j=0;j<patch.triple_stitches.size();j++){
            shared_ptr<TripleStitch> tripleStitch = patch.triple_stitches[j];
            continue;
            std::shared_ptr<CoarseTriangle> triangle =
                    patch.getCoarseTriangleWith(tripleStitch->patches[0].lock(),
                                        tripleStitch->patches[1].lock(),
                                        tripleStitch->patches[2].lock());
            if(triangle!=nullptr){
                //seems like we already have a triangle connecting these three
                //continue;
            }

            triangle = make_shared<CoarseTriangle>(tripleStitch->patches[0].lock(),
                    tripleStitch->patches[1].lock(),
                    tripleStitch->patches[2].lock());
            tripleStitch->patches[0].lock()->addCoarseTriangle(triangle);
            tripleStitch->patches[1].lock()->addCoarseTriangle(triangle);
            tripleStitch->patches[2].lock()->addCoarseTriangle(triangle);
            //also fill this triangle with everything thats needed
            //(new)

            newCoarseTriangles.push_back(triangle);

        }
        //patch.tripleStitchMutex.unlock();
        for(set<shared_ptr<MeshPatch>>::iterator it1 = neighbours.begin();
            it1!=neighbours.end();++it1){
            //cout <<" one " << (*it1)->id << endl;
            //continue;
            //find all the neighbours of the neighbours
            set<shared_ptr<MeshPatch>> n2eighbours = (*it1)->getNeighbours();
            for(set<shared_ptr<MeshPatch>>::iterator it2 = n2eighbours.begin();
                it2!=n2eighbours.end();++it2){
                //cout << " two " << (*it2)->id << endl;
                //test if we have a fitting triangle by comparing with all the neighbouring
                //iterate again over the first set of neighbours (O(n^3))
                for(set<shared_ptr<MeshPatch>>::iterator it3 = neighbours.begin();
                        it3!=neighbours.end();++it3){
                   // cout << " three " << (*it3)->id << endl;
                    if((*it3) == (*it2)){
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
                        cout << "found a neighbour" << endl;
#endif
                        //found one triangle neighbourhood
                        //the next step is to find one already existing triangle
                        std::shared_ptr<CoarseTriangle> triangle =
                                patch.getCoarseTriangleWith(patchesIn[i],
                                                    *it1,
                                                    *it2);
                        /*cout << "triangle created with ids:" << patchesIn[i]->id <<
                                " , " << (*it1)->id << " , " << (*it2)->id << endl;
                        */


                        if(triangle!=nullptr){
                            //seems like we already have a triangle connecting these three
                            continue;
                        }

                        triangle = make_shared<CoarseTriangle>(patchesIn[i],*it1,*it2);
                        /*if(!triangle->isConnectingSame3Patches(
                                    patchesIn[i],*it2,*it1)){
                            triangle->isConnectingSame3Patches( patchesIn[i],*it2,*it1);
                            cout << "why does this fail???? " << endl;
                            assert(0);

                        }*/
                        patchesIn[i]->addCoarseTriangle(triangle);
                        (*it1)->addCoarseTriangle(triangle);
                        (*it2)->addCoarseTriangle(triangle);
                        //also fill this triangle with everything thats needed
                        //(new)

                        newCoarseTriangles.push_back(triangle);


                    }

                }
            }
        }
    }

    cout << " LowDetailRenderer::addPatches right after creating the triangle list" << endl;


    //now we update the buffer. (create a completely new one which admittedly is wasteful)
    //TODO: create mechanism that doesn't require us to completely update the whole buffer

    vector<GpuCoarseVertex> newVertices(newPatches.size());
    vector<int> newVisible(newPatches.size());
    for(size_t i = 0;i<newPatches.size();i++){
        if(newPatches[i].expired()){
            newVisible[i]=0;
            continue;
        }
        shared_ptr<MeshPatch> patch = newPatches[i].lock();
        Vector3f p3 = patch->getPos();
        newVertices[i].p = Vector4f(p3[0],p3[1],p3[2],1.0f);
        newVertices[i].n = Vector4f(NAN,NAN,NAN,NAN);//placeholder
        newVertices[i].c = Vector4f(1,0,0,1);//red placeholder

        newVisible[i]=1;


    }


    vector<int> newIndices;
    for(size_t i = 0;i < newCoarseTriangles.size();i++){
        if(newCoarseTriangles[i].expired()){
            continue;
        }
        shared_ptr<CoarseTriangle> triangle = newCoarseTriangles[i].lock();
        triangle->flipToFacePos(camPos);
        if(triangle->isValid()){
            for(size_t j=0;j<3;j++){
                //iterate over points and add all the points to the index buffers.
                newIndices.push_back(triangle->patches[j].lock()->indexWithinCoarse);
            }
        }
    }
    int oldNrVertices=patches.size();
    int oldNrIndices= coarseTriangles.size()*3;

    //append the new triangles and patches to the list
    patches.insert(patches.end(),newPatches.begin(),newPatches.end());
    coarseTriangles.insert(coarseTriangles.end(),
                           newCoarseTriangles.begin(),
                           newCoarseTriangles.end());


    modifyingBuffers.lock();

    std::shared_ptr<GlCudaBuffer<int>> indBuf =
            indexBuffer;
    std::shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vertBuf =
            vertexBuffer;
    std::shared_ptr<GlCudaBuffer<int>> visBuf =
            visibilityBuffer;
    int nrInds;
    modifyingBuffers.unlock();






    bool newVertexBuffer = false;
    bool newIndexBuffer = false;
    if(vertBuf==nullptr){
        newVertexBuffer=true;
        //create a whole new buffer
        vertBuf = make_shared<GlCudaBuffer<GpuCoarseVertex>>(patches.size()*2);
        visBuf = make_shared<GlCudaBuffer<int>>(patches.size()*2);

    }else{
        if(vertBuf->nrElements < patches.size()){
            newVertexBuffer = true;
            //create a whole new buffer and copy the old stuff in
            std::shared_ptr<GlCudaBuffer<GpuCoarseVertex>> oldVertBuf =
                    vertBuf;
            vertBuf =
                    make_shared<GlCudaBuffer<GpuCoarseVertex>>(patches.size()*2);
            cudaMemcpy(vertBuf->cudaPtr,
                    oldVertBuf->cudaPtr,
                    oldVertBuf->nrElements*sizeof(GpuCoarseVertex),
                    cudaMemcpyDeviceToDevice);


            std::shared_ptr<GlCudaBuffer<int>> oldVisBuf =
                    visBuf;
            visBuf =
                    make_shared<GlCudaBuffer<int>>(patches.size()*2);
            cudaMemcpy(visBuf->cudaPtr,
                    oldVisBuf->cudaPtr,
                    oldVisBuf->nrElements*sizeof(int),
                    cudaMemcpyDeviceToDevice);

            cudaDeviceSynchronize();
            gpuErrchk( cudaPeekAtLastError() );

        }
    }

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );


    if(coarseTriangles.size()==0){
        //cout << "DEUBG: i am not sure if this does invalidate the whole structure" << endl;
        //return;
    }
    //upload novel data
    cudaMemcpy(&(vertBuf->cudaPtr[oldNrVertices]),
            &(newVertices[0]),
            newVertices.size()*sizeof(GpuCoarseVertex),
            cudaMemcpyHostToDevice);

    cudaMemcpy(&(visBuf->cudaPtr[oldNrVertices]),
            &(newVisible[0]),
            newVertices.size()*sizeof(int),
            cudaMemcpyHostToDevice);


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );


    if(indBuf==nullptr){
        newIndexBuffer = true;
        indBuf = make_shared<GlCudaBuffer<int>>(coarseTriangles.size()*3*2);
    }else{
        if(indBuf->nrElements < coarseTriangles.size()*3 ){//TODO: delete this *2
            newIndexBuffer=true;
            std::shared_ptr<GlCudaBuffer<int>> oldIndBuf =
                    indBuf;
            //create a new buffer of bigger size:
            indBuf = make_shared<GlCudaBuffer<int>>(coarseTriangles.size()*3*2);

            //copy over the old data
            cudaMemcpy(indBuf->cudaPtr,
                    oldIndBuf->cudaPtr,
                    oldIndBuf->nrElements*sizeof(int),
                    cudaMemcpyDeviceToDevice);

        }
    }
    //upload the new data
    cudaMemcpy(&(indBuf->cudaPtr[oldNrIndices]),
            &(newIndices[0]),
            newIndices.size()*sizeof(int),
            cudaMemcpyHostToDevice);
    nrInds = coarseTriangles.size()*3;


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );








    //setup the gpu to calculate the average colors of the vertices
    vector<CalcMeanColorDescriptor> descriptors;//(patchesIn.size());
    for(size_t i=0;i<patchesIn.size();i++){
        MeshPatch &patch = *(patchesIn[i].get());
        if(patch.tex_patches.size() == 0){
            //if there is no color for the texture we don't update it.
            continue;
        }
        MeshTexture &tex = *(patch.tex_patches[0].get());

        shared_ptr<MeshTextureGpuHandle> texGpuHandle = tex.gpu.lock();
        CalcMeanColorDescriptor desc;//&desc = descriptors[i];

        if(patch.tex_patches.size()==0){
            continue;
        }
        if(tex.gpu.lock() ==nullptr){
            continue;
        }
        cv::Rect2f rect = texGpuHandle->tex->getRect();
        desc.color = texGpuHandle->tex->getCudaSurfaceObject();
        desc.scale = 1.0f;
        desc.hdr = false;
        desc.x = rect.x;
        desc.y = rect.y;
        desc.width = rect.width;
        desc.height = rect.height;
        desc.vert_ind = patch.indexWithinCoarse;
        descriptors.push_back(desc);

    }

    //use the kernel to update the vertices on the gpu
    calcMeanColor(descriptors,vertBuf->cudaPtr,0);



    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );


    modifyingBuffers.lock();
    nrIndices = nrInds;

    //set the buffer references for rendering
    if(newVertexBuffer || newIndexBuffer){
        //but only do this if one of the buffers had to be resetted

        indexBuffer = indBuf;
        vertexBuffer= vertBuf;
        visibilityBuffer = visBuf;

        newBuffers = true;

    }
    modifyingBuffers.unlock();


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

}

void LowDetailRenderer::downloadCurrentGeometry(std::vector<GpuCoarseVertex> &vertices,std::vector<int> &indices){
    //while(1);

    modifyingBuffers.lock();

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );


    indices.resize(coarseTriangles.size()*3);//indexBuffer->nrElements);



    cudaMemcpy(&(indices[0]),//dst
               indexBuffer->cudaPtr,//src
               coarseTriangles.size()*sizeof(int)*3,
               cudaMemcpyDeviceToHost);


    //AAAAAAAAAAAH EVERYTHING HERE IS SUPERAWFUL!!!!!!!!

    int count = coarseTriangles.size();
    int count2 = indices.size();
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    //TODO: it might be that some of these patches have gottin invalid
    //then we really need to change stuff
    int vertexCount = patches.size();
    vertices.resize(vertexCount);


    cudaMemcpy(&(vertices[0]),//dst
               vertexBuffer->cudaPtr,//src
               vertexCount*sizeof(GpuCoarseVertex),
               cudaMemcpyDeviceToHost);


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    modifyingBuffers.unlock();

}

void LowDetailRenderer::updateColorForPatches(std::vector<std::shared_ptr<MeshPatch> > &patchesIn)
{
    modifyingBuffers.lock();

    std::shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vertBuf =
            vertexBuffer;
    if(vertBuf == nullptr){
        modifyingBuffers.unlock();
        return;
    }
    //return;
    //prevent the render thread from swapping buffers
    bool newBuffersIntermediate=newBuffers;
    newBuffers=false;

    //we should somehow prevent the whole thing from creating new buffers.
    //especially deleting

    //setup the gpu to calculate the average colors of the vertices
    vector<CalcMeanColorDescriptor> descriptors;
    for(size_t i=0;i<patchesIn.size();i++){
        MeshPatch &patch = *(patchesIn[i].get());
        CalcMeanColorDescriptor desc;//&desc = descriptors[i];

        if(patch.tex_patches.size()==0){
            continue;
        }
        if(patch.indexWithinCoarse==-1){
            continue;//don't do anything if the patch is not part
            //of the coarse representation yet
        }
        MeshTexture &tex = *(patch.tex_patches[0].get());
        shared_ptr<MeshTextureGpuHandle> texGpuHandle = tex.gpu.lock();
        if(texGpuHandle->tex == nullptr){
            //continue;//actually this should not happen but in case it does we
            //want to prevent a crash
        }
        cv::Rect2f rect = texGpuHandle->tex->getRect();
        desc.color = texGpuHandle->tex->getCudaSurfaceObject();
        desc.scale = 1.0f;
        desc.hdr = false;
        desc.x = rect.x;
        desc.y = rect.y;
        desc.width = rect.width;
        desc.height = rect.height;
        desc.vert_ind = patch.indexWithinCoarse;
        descriptors.push_back(desc);

    }


    //use the kernel to update the vertices on the gpu
    calcMeanColor(descriptors,vertBuf->cudaPtr,0);



    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    newBuffers=newBuffersIntermediate;
    modifyingBuffers.unlock();
}

void LowDetailRenderer::renderExceptForActiveSets(std::vector<std::shared_ptr<ActiveSet> > &sets,
                                                  Eigen::Matrix4f proj,
                                                  Eigen::Matrix4f _camPose)
{
    GLUtils::checkForOpenGLError("[LowDetailRenderer::renderExceptForActiveSets] "
                                      "At the beginning.");
    shader->use();

    modifyingBuffers.lock();


    //prevent these buffers from beeing erased
    std::shared_ptr<GlCudaBuffer<int>> indBuf =
            indexBuffer;
    std::shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vertBuf =
            vertexBuffer;
    std::shared_ptr<GlCudaBuffer<int>> visBuf =
            visibilityBuffer;
    int nrInds=nrIndices;

    if(newBuffers){
        //copy the new buffers over the old ones
        //and also delete the old ones
        /*swap(indexBuffer[0],indexBuffer[1]);
        swap(vertexBuffer[0],vertexBuffer[1]);
        swap(vertexGpuPtr[0],vertexGpuPtr[1]);
        swap(visibleVertexBuffer[0],visibleVertexBuffer[1]);
        swap(nrIndices[0],nrIndices[1]);*/
        /*
        swap(vertexBuffers[0],vertexBuffers[1]);
        swap(visibilityBuffers[0],visibilityBuffers[1]);
        swap(indexBuffers[0],indexBuffers[1]);
        swap(nrIndices[0],nrIndices[1]);
        */



        //todo: delete the old buffers


        //adapt (or create) the VAO
        if(VAO == 0){
            glGenVertexArrays(1,&VAO);
        }
        glBindVertexArray(VAO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
                     indBuf->glName);
        glBindBuffer(GL_ARRAY_BUFFER,
                     vertBuf->glName);
        //pos
        glVertexAttribPointer(
            0,                                // attribute
            4,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            sizeof(GpuCoarseVertex),                                // stride (0 should work as well)
            (void*)0                          // array buffer offset
        );
        glEnableVertexAttribArray(0);
        //normal
        glVertexAttribPointer(
            1,                                // attribute
            4,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            sizeof(GpuCoarseVertex),                                // stride (0 should work as well)
            (void*)(sizeof(Vector4f)*1)                          // array buffer offset
        );
        glEnableVertexAttribArray(1);
        //color
        glVertexAttribPointer(
            2,                                // attribute
            4,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            sizeof(GpuCoarseVertex),                                // stride (0 should work as well)
            (void*)(sizeof(Vector4f)*2)                         // array buffer offset
        );
        glEnableVertexAttribArray(2);

        glBindBuffer(GL_ARRAY_BUFFER,
                     visBuf->glName);
        glVertexAttribPointer(
            3,                                // attribute
            1,                                // size
            GL_INT,                         // type
            GL_FALSE,                         // normalized?
            sizeof(int),                                // stride (0 should work as well)
            (void*)0                        // array buffer offset
        );
        glEnableVertexAttribArray(3);





        newBuffers=false;
    }

    modifyingBuffers.unlock();
    if(VAO==0){
        return;
    }


    //make the triangles visible/invisible
    //size_t nrDisabledPatches=0;
    std::vector<int> disablePatches;
    int debugMax=0;
    for(size_t i=0;i<sets.size();i++){
        std::shared_ptr<ActiveSet> &aset = sets[i];
        if(aset == nullptr){
            continue;
        }
        //nrDisabledPatches += aset->currentlyAdded.size();
        for(size_t j=0;j<aset->retainedMeshPatchesCpu.size();j++){
            shared_ptr<MeshPatch> patch = aset->retainedMeshPatchesCpu[j];
            //shared_ptr<MeshPatchGpuHandle> gpu = aset->retainedMeshPatchesCpu[j]->gpu.lock;

            disablePatches.push_back(patch->indexWithinCoarse);
            if(debugMax<patch->indexWithinCoarse){
                debugMax=patch->indexWithinCoarse;
            }
        }
    }

    for(size_t i=0;i < invisibleInLastFrame.size();i++){
        if(invisibleInLastFrame[i]>=visBuf->nrElements){
            assert(0);
        }
    }
    for(size_t i=0;i < disablePatches.size();i++){
        if(disablePatches[i]>=visBuf->nrElements){
            assert(0);
        }
    }
    coarseUpdateVisibility(invisibleInLastFrame,disablePatches,visBuf->cudaPtr);
    invisibleInLastFrame = disablePatches;




    //disable vertices included within this active set
    //(do this with a cuda kernel)

    glBindVertexArray(VAO);

    Matrix4f mvp = proj*_camPose;
    glUniformMatrix4fv(0,1,false,(float*)&mvp);


    GLUtils::checkForOpenGLError("[LowDetailRenderer::renderExceptForActiveSets] "
                                      "Error at setting up the low detail rendering.");

    //render the low poly version of the map
    //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );//debug wireframe
    glDrawElements(
                GL_TRIANGLES,//GL_TRIANGLES,
                nrInds,
                GL_UNSIGNED_INT,
                (void*)0);
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );//back from debug
    /*if(nrIndices[0]>0){
        debugShader->use();
        glUniformMatrix4fv(0,1,false,(float*)&mvp);
        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS,
                     0,
                     debugNrVertices);

    }*/
    glFinish();//DEBUG: this should not be necessary.
    //check for opengl errors
    GLUtils::checkForOpenGLError("[LowDetailRenderer::renderExceptForActiveSets] "
                                      "Error at rendering coarse reconstruction.");

}

void LowDetailRenderer::updateMaskForActiveSets(std::vector<std::shared_ptr<ActiveSet> > &sets)
{

}

void LowDetailRenderer::renderColor(Matrix4f proj, Matrix4f _camPose)
{

    modifyingBuffers.lock();


    //prevent these buffers from beeing erased
    std::shared_ptr<GlCudaBuffer<int>> indBuf =
            indexBuffer;
    (void)indBuf;
    std::shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vertBuf =
            vertexBuffer;
    (void)vertBuf;
    std::shared_ptr<GlCudaBuffer<int>> visBuf =
            visibilityBuffer;
    (void)visBuf;//silence unused warning
    int nrInds=nrIndices;

    modifyingBuffers.unlock();
    //disable vertices included within this active set
    //(do this with a cuda kernel)

    if(VAO==0){
        return;
    }

    glBindVertexArray(VAO);
    shader->use();


    Matrix4f mvp = proj*_camPose;
    glUniformMatrix4fv(0,1,false,(float*)&mvp);


    GLUtils::checkForOpenGLError("[LowDetailRenderer::renderExceptForActiveSets] "
                                      "Error at setting up the low detail rendering.");

    //render the low poly version of the map
    //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );//debug wireframe
    glDrawElements(
                GL_TRIANGLES,//GL_TRIANGLES,
                nrInds,
                GL_UNSIGNED_INT,
                (void*)0);
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );//back from debug

    glFinish();//DEBUG: this should not be necessary.
    //check for opengl errors
    GLUtils::checkForOpenGLError("[LowDetailRenderer::renderColor] "
                                      "Error at rendering coarse reconstruction.");

}

void LowDetailRenderer::renderGeometry(Matrix4f proj, Matrix4f _camPose)
{
    modifyingBuffers.lock();


    //prevent these buffers from beeing erased
    std::shared_ptr<GlCudaBuffer<int>> indBuf =
            indexBuffer;
    (void)indBuf;
    std::shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vertBuf =
            vertexBuffer;
    (void)vertBuf;
    std::shared_ptr<GlCudaBuffer<int>> visBuf =
            visibilityBuffer;
    (void)visBuf;//silence unused warning
    int nrInds=nrIndices;

    modifyingBuffers.unlock();

    if(VAO==0){
        return;
    }
    glBindVertexArray(VAO);
    geometryShader->use();

    Matrix4f mvp = proj*_camPose;
    glUniformMatrix4fv(0,1,false,(float*)&mvp);

    //render the low poly version of the map
    //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );//debug wireframe
    glDrawElements(
                GL_TRIANGLES,//GL_TRIANGLES,
                nrInds,
                GL_UNSIGNED_INT,
                (void*)0);
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );//back from debug

    glFinish();//DEBUG: this should not be necessary.
}

/*
void LowDetailRenderer::updateAvgColorForPatches(std::vector<std::shared_ptr<MeshPatch> > &patches)
{
    vector<CalcMeanColorDescriptor> descriptors(patches.size());
    for(size_t i=0;i<patches.size();i++){
        MeshPatch &patch = *(patches[i].get());
        CalcMeanColorDescriptor &desc = descriptors[i];

        MeshTexture tex = *(patch.texPatches[0].get());
        cv::Rect2f rect = tex.getSourceRect();
        desc.color = tex.getSourceTexPatch()->getTex()->getCudaSurfaceObject();
        desc.scale = 1.0f;
        desc.hdr = false;
        desc.x = rect.x;
        desc.y = rect.y;
        desc.width = rect.width;
        desc.height = rect.height;
        desc.vertInd = patch.indexWithinCoarse;

    }

    //use the kernel to update the vertices on the gpu
    calcMeanColor(descriptors,vertexGpuPtr[1],0);

}
*/
/*
bool LowDetailRenderer::submit()
{
    newBuffers=true;
}*/

bool CoarseTriangle::isValid()
{
    for(size_t i = 0;i<3;i++){
        if( patches[i].expired() ){//||
                //doubleStitches[i].expired() ||
                //tripleStitches[i].expired()){
            return false;
        }
    }
    return true;
}

CoarseTriangle::CoarseTriangle(std::shared_ptr<MeshPatch> p1, std::shared_ptr<MeshPatch> p2, std::shared_ptr<MeshPatch> p3)
{
    patches[0] = p1;
    patches[1] = p2;
    patches[2] = p3;
    //shared_ptr<MeshPatch> p[3]={p1,p2,p3};

    doubleStitches[0] = p1->getDoubleStitchTo(p2);
    doubleStitches[1] = p1->getDoubleStitchTo(p3);
    doubleStitches[2] = p2->getDoubleStitchTo(p3);

    tripleStitches[0] = p1->getTripleStitchTo(p2);
    tripleStitches[1] = p1->getTripleStitchTo(p3);
    tripleStitches[2] = p2->getTripleStitchTo(p3);
}

CoarseTriangle::~CoarseTriangle()
{

}

bool CoarseTriangle::isConnectingSame3Patches(std::shared_ptr<MeshPatch> p1,
                                              std::shared_ptr<MeshPatch> p2,
                                              std::shared_ptr<MeshPatch> p3){
    if(!isValid()){
        return false;
    }
    int indices[] = {0,1,2};
    std::sort(indices,indices+3);
    do{
        if(     patches[indices[0]].lock() == p1 &&
                patches[indices[1]].lock() == p2 &&
                patches[indices[2]].lock() == p3){
            return true;
        }
    }while(std::next_permutation(indices,indices+3));
    return false;
}

bool CoarseTriangle::flipToFacePos(Eigen::Vector3f pos) {
    Vector3f toCamera =  pos - patches[0].lock()->getPos();

    Vector3f v1 = patches[1].lock()->getPos() - patches[0].lock()->getPos();
    Vector3f v2 = patches[2].lock()->getPos() - patches[0].lock()->getPos();
    if(v1.cross(v2).dot(toCamera)<0){
        std::swap(patches[1],patches[2]);
        std::swap(doubleStitches[1],doubleStitches[2]);
        std::swap(tripleStitches[1],tripleStitches[2]);

    }
}

//template<class T>
//template<class T>
//template<class T>
void LowDetailPoint::addCoarseTriangle(std::shared_ptr<CoarseTriangle> coarseTriangle)
{
    coarseTriangleMutex.lock();
    triangleWithinNeighbours.push_back(coarseTriangle);
    coarseTriangleMutex.unlock();
}

std::shared_ptr<CoarseTriangle> LowDetailPoint::getCoarseTriangleWith(std::shared_ptr<MeshPatch> p1, std::shared_ptr<MeshPatch> p2, std::shared_ptr<MeshPatch> p3)
{

    coarseTriangleMutex.lock();
    for(size_t i=0;i<triangleWithinNeighbours.size();i++){
        std::shared_ptr<CoarseTriangle> triangle = triangleWithinNeighbours[i];
        if(triangle->isConnectingSame3Patches(p1,p2,p3)){
            coarseTriangleMutex.unlock();
            return triangle;
        }
    }
    coarseTriangleMutex.unlock();
    return nullptr;
}

//TODO: think about when we want to call this function because at some point it might become necessary
void LowDetailPoint::cleanupCoarseTriangles()
{
    coarseTriangleMutex.lock();
    assert(0); //not implemented yet
    coarseTriangleMutex.unlock();
}

template<typename T>
GlCudaBuffer<T>::GlCudaBuffer(size_t size)
{
    this->nrElements=size;
    glGenBuffers(1,&glName);
    glBindBuffer(GL_ARRAY_BUFFER,glName);
    glBufferData(GL_ARRAY_BUFFER,size*sizeof(T),0,GL_DYNAMIC_DRAW);

    cudaError_t error_test;
    error_test = cudaGraphicsGLRegisterBuffer(&cudaResource,glName,cudaGraphicsMapFlagsNone);//TODO: test these
    gpuErrchk( error_test );
    error_test = cudaGraphicsMapResources(1,&cudaResource);//stream 0
    gpuErrchk( error_test );
    size_t bytes;
    error_test = cudaGraphicsResourceGetMappedPointer((void**)&cudaPtr,&bytes,cudaResource);
    gpuErrchk( error_test );
    //error_test = cudaGraphicsUnmapResources(1,&indCudaGraphicsResource);//stream0
    //gpuErrchk( error_test );

    //also upload the data
    //error_test = cudaMemcpy(indGpu,&indices[0],indices.size()*sizeof(int),cudaMemcpyHostToDevice);
    //gpuErrchk( error_test );
    //debugNrVertices = vertices.size();
}
template<typename T>
GlCudaBuffer<T>::~GlCudaBuffer()
{
    cudaGraphicsUnmapResources(1,&cudaResource);
    cudaGraphicsUnregisterResource(cudaResource);
    glDeleteBuffers(1,&glName);


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

}
