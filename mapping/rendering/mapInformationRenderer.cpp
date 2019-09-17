#include "mapInformationRenderer.h"

#include "../meshReconstruction.h"
#include "../gpu/ActiveSet.h"

#include <opencv2/opencv.hpp>

#include <glUtils.h>


const std::string information_frag =
#include "information.frag"//todo: update this file with stuff from renderDepth.frag
;
const std::string information_vert =
#include "datastructure.glsl"
#include "information.vert"
;

const std::string coordinates_frag =
#include "coordinates.frag"
;

const std::string coordinates_vert =
#include "datastructure.glsl"
#include "coordinates.vert"
;


const std::string triangleRefDepth_frag =
        #include "triangleRefDepth.frag"
        ;

const std::string triangleRefDepth_geom =
        #include "datastructure.glsl"
        #include "triangleRefDepth.geom"
        ;
const std::string triangleRefDepth_vert =
        #include "datastructure.glsl"
        #include "triangleRefDepth.vert"
        ;


const std::string unifiedInfo_frag =
//#include "datastructure.glsl"
#include "unifiedInfo.frag"
;

const std::string unifiedInfo_geom =
#include "datastructure.glsl"
#include "unifiedInfo.geom"
;
const std::string unifiedInfo_vert =
#include "datastructure.glsl"
#include "unifiedInfo.vert"
;

using namespace gfx;
using namespace std;
using namespace Eigen;
using namespace cv;

std::mutex MapInformationRenderer::shaderMutex;
std::weak_ptr<gfx::GLSLProgram> MapInformationRenderer::s_depthProgram;
std::weak_ptr<gfx::GLSLProgram> MapInformationRenderer::s_triangleReferenceProgram;
std::weak_ptr<gfx::GLSLProgram> MapInformationRenderer::s_triangleRefDepthProgram;

MapInformationRenderer::MapInformationRenderer(int width, int height)
{
    m_width=width;
    m_height=height;

}

MapInformationRenderer::~MapInformationRenderer()
{

}

void MapInformationRenderer::initInContext(int width, int height, MeshReconstruction *map)
{
    m_map = map;
    m_width=width;
    m_height=height;
    initInContext();
}

void MapInformationRenderer::initInContext()
{
    //assert(0); //make this multithreading capable: (it depends on the VAO i think)
    //but also on the texture..... both of them need to be thread specific

    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] start");

    MapInformationRenderer::shaderMutex.lock();
    //render depth shader
    if(MapInformationRenderer::s_depthProgram.use_count()){//if the shader exists already copy the reference
        m_depthProgram = MapInformationRenderer::s_depthProgram.lock();

    }else{//otherwise create a new shader

        m_depthProgram = std::shared_ptr<GLSLProgram>(new GLSLProgram());
        m_depthProgram->compileShader( information_frag,
                                 gfx::GLSLShader::GLSLShaderType::FRAGMENT,
                                 "information.frag" );
        m_depthProgram->compileShader( information_vert,
                                 gfx::GLSLShader::GLSLShaderType::VERTEX,
                                 "information.vert" );
        m_depthProgram->link();
        MapInformationRenderer::s_depthProgram = m_depthProgram;
    }
    MapInformationRenderer::shaderMutex.unlock();


    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] after setting up information shader");


    /**
     * This shader is supposed to create texture coordinates and indices of the triangles  for a texture.
     * the indices for the triangles as well as the indices
     */


    MapInformationRenderer::shaderMutex.lock();
    if(MapInformationRenderer::s_triangleReferenceProgram.use_count()){
        m_triangleReferenceProgram = MapInformationRenderer::s_triangleReferenceProgram.lock();
    }else{

        m_triangleReferenceProgram = std::shared_ptr<GLSLProgram>(new GLSLProgram());
        m_triangleReferenceProgram->compileShader( coordinates_frag,
                                 gfx::GLSLShader::GLSLShaderType::FRAGMENT,
                                 "coordinates.frag" );
        m_triangleReferenceProgram->compileShader( coordinates_vert,
                                 gfx::GLSLShader::GLSLShaderType::VERTEX,
                                 "coordinates.vert" );
        m_triangleReferenceProgram->link();
        MapInformationRenderer::s_triangleReferenceProgram = m_triangleReferenceProgram;
    }
    MapInformationRenderer::shaderMutex.unlock();

    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] after setting up coordinates shader");

    MapInformationRenderer::shaderMutex.lock();

    if(MapInformationRenderer::s_triangleRefDepthProgram.use_count()){
        triangleRefDepthProg =
                MapInformationRenderer::s_triangleRefDepthProgram.lock();
    }else{
        //create the shader i was looking for
        triangleRefDepthProg = std::shared_ptr<GLSLProgram>(new GLSLProgram());
        triangleRefDepthProg->compileShader(triangleRefDepth_frag,
                                            gfx::GLSLShader::GLSLShaderType::FRAGMENT,
                                            "triangleRefDepth.frag" );
        /* //do not utilize the geometry shader yet
        triangleRefDepthProg->compileShader(triangleRefDepth_geom,
                                            gfx::GLSLShader::GLSLShaderType::GEOMETRY,
                                            "triangleRefDepth.geom" );
        */
        triangleRefDepthProg->compileShader(triangleRefDepth_vert,
                                            gfx::GLSLShader::GLSLShaderType::VERTEX,
                                            "triangleRefDepth.vert" );
        /*
        cout << "start of this stupid shader program:" << endl;
        cout<< triangleRefDepth_vert << endl;
        cout << "end of this stupid shader program" << endl;
        */

        triangleRefDepthProg->link();
        MapInformationRenderer::s_triangleRefDepthProgram = triangleRefDepthProg;


    }
    MapInformationRenderer::shaderMutex.unlock();


    //TODO: get rid of these overflowing error checks
    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] after setting up triangle Ref shader");

    unifiedInfoProg = make_shared<GLSLProgram>();

    assert(gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] before setting up unified info shader")==GL_NO_ERROR);


    unifiedInfoProg->compileShader(unifiedInfo_frag,
            gfx::GLSLShader::GLSLShaderType::FRAGMENT,
            "unifiedInfo.frag");

    assert(gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] during setting up unified info shader")==GL_NO_ERROR);

    /*
    unifiedInfoProg->compileShader(unifiedInfo_geom,
            gfx::GLSLShader::GLSLShaderType::GEOMETRY,
            "unifiedInfo.geom");
            */
    unifiedInfoProg->compileShader(unifiedInfo_vert,
            gfx::GLSLShader::GLSLShaderType::VERTEX,
            "unifiedInfo.vert");

    assert(gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] after setting up unified info shader")==GL_NO_ERROR);

    unifiedInfoProg->link();


    //TODO: remove this
    //cout << unifiedInfo_frag << endl;

    //cout << unifiedInfo_vert << endl;

    assert(gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] after linking up unified info shader")==GL_NO_ERROR);


    //after creating the shader which should only be done once we
    //create the struct
    //**************************************NEW*******************************
    thread::id id = this_thread::get_id();

    //if this thread does not have the texture and stuff (its very certain it has not)
    //we create all the resources
    if(perThreadGlObjects.count(id) == 0){
        PerThread pt;

        //now do everything that also is done below:
        {
            gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] before setting up framebuffers");
            //FBO creation:
            glGenFramebuffers(1, &pt.depthFBO);
            glBindFramebuffer(GL_FRAMEBUFFER, pt.depthFBO);


            //texture creation
            //depth:

            pt.depthTexture =
                    std::make_shared<gfx::GpuTex2D>(m_map->garbageCollector,GL_RGBA32F,GL_RGBA,GL_FLOAT,m_width,m_height,false);
            pt.depthTexture->name = "perThread depth texture";
            //debug:
            pt.stdTexture =
                    std::make_shared<gfx::GpuTex2D>(m_map->garbageCollector,GL_RGBA32F,GL_RGBA,GL_FLOAT,m_width,m_height,false);
            pt.stdTexture->name = "perThread std Texture";


            glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, pt.depthTexture->getGlName(), 0);

            glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, pt.stdTexture->getGlName(), 0);

            gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] setting up depth buffer");

            //setting up the depth buffer
            glGenTextures(1,&pt.depthBufferTex);
            glBindTexture(GL_TEXTURE_2D,pt.depthBufferTex);
            glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT32, m_width, m_height, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
            glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,pt.depthBufferTex,0);
            //TODO: (or to think of)binding the depth buffer to cuda
            //http://stackoverflow.com/questions/32611002/opengl-depth-buffer-to-cuda
            //https://devtalk.nvidia.com/default/topic/877969/opengl-z-buffer-to-cuda/

            gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] setting up z buffer");

            //TODO: it might not work if the

            GLenum DrawBuffers[2] = {GL_COLOR_ATTACHMENT0,GL_COLOR_ATTACHMENT1};
            glDrawBuffers(2, DrawBuffers); // "1" is the size of DrawBuffers

            gfx::GLUtils::checkOpenGLFramebufferStatus("Initializing Render map Informations");
            gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] setting up framebuffer");

            //todo add depth buffer...
            glBindFramebuffer(GL_FRAMEBUFFER,0);//unbind the framebuffer
            gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] while setting up framebuffer");


            //create the VAO just because it is necessary for rendering:
            glGenVertexArrays(1,&pt.depthVAO);
            glBindVertexArray(pt.depthVAO);
        }




        //Create textures for the combined/unified rendering stuff.
        {
            //TODO: some of these textures are not needed or have wrong resolution
            glGenFramebuffers(1, &pt.combinedFBO);
            glBindFramebuffer(GL_FRAMEBUFFER, pt.combinedFBO);


            pt.zTexture =
                    std::make_shared<gfx::GpuTex2D>(m_map->garbageCollector,GL_R32F,GL_RED,GL_FLOAT,m_width,m_height,false);
            pt.zTexture->name = "per thread zTexture";
            pt.colorTexture =
                    std::make_shared<gfx::GpuTex2D>(m_map->garbageCollector,GL_RGBA32F,GL_RGBA,GL_FLOAT,m_width,m_height,false);
            pt.colorTexture->name = "per thread colorTexture";
            pt.normalTexture =
                    std::make_shared<gfx::GpuTex2D>(m_map->garbageCollector,GL_RGBA32F,GL_RGBA,GL_FLOAT,m_width,m_height,false);
            pt.normalTexture->name = "per thread normalTexture";
            pt.labelTexture = //make this an int32.... this is going to be hard enough
                    std::make_shared<gfx::GpuTex2D>(m_map->garbageCollector,GL_RGBA32F,GL_RGBA,GL_FLOAT,m_width,m_height,false);
            pt.labelTexture->name = "per thread label texture";
            glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, pt.zTexture->getGlName(), 0);
            glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, pt.colorTexture->getGlName(), 0);
            glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, pt.normalTexture->getGlName(), 0);
            glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, pt.labelTexture->getGlName(), 0);




            glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,pt.depthBufferTex,0);
            GLenum DrawBuffers[4] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1,
                                     GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3};
            glDrawBuffers(4, DrawBuffers); // "1" is the size of DrawBuffers



            glGenVertexArrays(1,&pt.combinedVAO);
            glBindVertexArray(pt.combinedVAO);
        }











        perThreadGlObjects[id]=pt;

    }
    return;


}

std::shared_ptr<GpuTex2D> MapInformationRenderer::getDepthTexture()
{
    return perThreadGlObjects[this_thread::get_id()].depthTexture;
}

std::shared_ptr<GpuTex2D> MapInformationRenderer::getStdTexture()
{
    return perThreadGlObjects[this_thread::get_id()].stdTexture;
}

void MapInformationRenderer::renderDepth(ActiveSet *activeSet, Matrix4f projection, Matrix4f pose)
{

    //cout << "started rendering the active set" << endl;
    PerThread pt = perThreadGlObjects[this_thread::get_id()];
    //activate VBO
    glBindFramebuffer(GL_FRAMEBUFFER,pt.depthFBO);

    //there propably is a fbo bound somewhere already


    glViewport(0,0,m_width,m_height);

    glClearColor(NAN,NAN,NAN,NAN);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    m_depthProgram->use();

    //after setting up the uniforms we set up all the possible
    Matrix4f _pose=pose.inverse();
    Matrix4f _projection=projection.inverse();
    glUniformMatrix4fv(0,1,false,(GLfloat*)&_pose);
    glUniformMatrix4fv(1,1,false,(GLfloat*)&projection);
    glUniformMatrix4fv(4,1,false,(GLfloat*)&_projection);//for as soon as i added this...

    //uint32_t maxNrTriangles = activeSet->gpuGeomStorage->m_maxNrTrianglesPerPatch;
    struct Uniforms{
        int maxNrTexPoints;
        int maxNrVertices;
    };
    Uniforms uniforms;
    //uniforms.maxNrVertices = activeSet->gpuGeomStorage->m_maxNrVerticesPerPatch;
    //uniforms.maxNrTexPoints= activeSet->gpuGeomStorage->m_maxNrTexCoordsPerPatch;
    //cout << "this is unused" << endl;
    //glUniform1i(8,maxNrTriangles); // might be that this is not needed anymore
                                    //might also be that we need much more than that ( see texture coordinates)
    //glUniform2i(8,uniforms.maxNrTriangles,uniforms.maxNrTexPoints);
    //glUniform1i(2,uniforms.maxNrTexPoints);//debug: womehow this works but the line above doesn't
    //glUniform1i(2,uniforms.maxNrTexPoints);
    //glUniform1i(3,uniforms.maxNrVertices);


    if(activeSet->retainedMeshPatches.size()==0){
        //the active set is empty
        glFinish();
        return;
    }
    //the vertex buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
                     activeSet->gpuGeomStorage->vertexBuffer->getGlName());

    //bind texture coordinates
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
                     activeSet->gpuGeomStorage->texPosBuffer->getGlName());

    //the triangle buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
                     activeSet->gpuGeomStorage->triangleBuffer->getGlName());

    //the patch information
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
                     activeSet->gpuGeomStorage->patchInfoBuffer->getGlName());


    activeSet->drawEverything();
    gfx::GLUtils::checkForOpenGLError("[MapInformationRenderer::renderDepth] "
                                      "after issuing all render commands");

    glFinish();

/*
    //cout << "rendering " << activeSet->currentlyAdded.size() << " meshPatches" << endl;
    //first iterate over all the mesh patches:
    for(size_t i=0;i<activeSet->retainedMeshPatches.size();i++){
        MeshPatchGpuHandle &currentPatch =
                *activeSet->retainedMeshPatches[0];

        //first check that all the textures are resident
        if(currentPatch.geomTex!=nullptr){
            uint64_t texPtr =
                    currentPatch.geomTex->sourceTex->getTex()->getGlHandle();
            if(!glIsTextureHandleResidentARB(texPtr)){
                glMakeTextureHandleResidentARB(texPtr);
            }
        }
        for(size_t j=0;j<currentPatch.texCount;j++){
            if(currentPatch.texs[j]!=nullptr){
                uint64_t texPtr =
                        currentPatch.texs[j]->sourceTex->getTex()->getGlHandle();
                if(!glIsTextureHandleResidentARB(texPtr)){
                    glMakeTextureHandleResidentARB(texPtr);
                }
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
    for(size_t i=0;i<activeSet->retainedStitches.size();i++){
        StitchGpuHandle &currentStitch = *activeSet->retainedStitches[i];

        int slot = currentStitch.triangles->getStartingIndex();
        int count = currentStitch.triangles->getSize();
        glDrawArrays(GL_TRIANGLES,
                     slot*3,
                     count*3);
    }
    */
}

/*
Mat RenderMapInformations::renderColorMapForImage(std::shared_ptr<SharedImage> image)
{
    cout << "[RenderMapInformations::renderColorMapForImage] i hope you are not using this, it is not doing anything."<<endl;
    Size2i size = image->getResolution();
    Mat result(size.height,size.width,CV_8UC4);
    for(int i=0;i<image->texPatches.size();i++){
        shared_ptr<TexturePatch> texPatch=image->texPatches[i].lock();
    }
    return result;
}
*/
void MapInformationRenderer::bindRenderTriangleReferenceProgram()
{
    //activate the shader.
    /*GLuint debugVAO;
    glGenVertexArrays(1,&debugVAO);
    glBindVertexArray(debugVAO);
    */


    m_triangleReferenceProgram->use();
    gfx::GLUtils::checkForOpenGLError("[MapInformationRenderer::bindRenderTriangleReferenceProgram] "
                                      "Initializing the triangle reference program..");


    //the vertex buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
                     m_map->m_gpuGeomStorage.vertexBuffer->getGlName());

    //bind texture coordinates
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
                     m_map->m_gpuGeomStorage.texPosBuffer->getGlName());

    //the triangle buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
                     m_map->m_gpuGeomStorage.triangleBuffer->getGlName());

    //the patch information
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
                     m_map->m_gpuGeomStorage.patchInfoBuffer->getGlName());

    gfx::GLUtils::checkForOpenGLError("[MapInformationRenderer::bindRenderTriangleReferenceProgram] "
                                      "Binding the buffers.");



    //bind the

    //set the uniforms needed to render this stuff.

    //int maxNrTexPoints = activeSet->gpuGeomStorage->m_maxNrTexCoordsPerPatch;
    //glUniform1i(0,maxNrTexPoints);
}

void MapInformationRenderer::renderTriangleReferencesForPatch(ActiveSet *activeSet, shared_ptr<MeshPatch> &patch,
                                                              shared_ptr<MeshTexture> &targetTexture)
{


    shared_ptr<MeshTextureGpuHandle> gpuTex= targetTexture->gpu.lock();
    if(gpuTex==nullptr){
        cout << "[MapInformationRenderer::renderTriangleReferencesForPatch] "
                "There is no texture on the gpu" << endl;
        assert(0);
    }
    //clear the list of dependencies since we are setting up a new one
    gpuTex->refTexDependencies.clear();
    Rect2i r = targetTexture->getLookupRect();
    glBindFramebuffer(GL_FRAMEBUFFER, gpuTex->refTex->getFBO());

    gfx::GLUtils::checkOpenGLFramebufferStatus("ScaleableMap::finalizeGeomTexOfNewPatches");
    glEnable(GL_SCISSOR_TEST);
    glScissor(r.x,r.y,r.width,r.height);
    glViewport(r.x,r.y,r.width,r.height);


    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesForPatch]");
    int minus1i = -1;
    float minus1 = *((float*)(&minus1i));//convert the -1 to a float in a bitwise fashion
    //(reinterpret cast)

    glClearColor(minus1,1,0,0);//DEBUG: paint it green
    glClear(GL_COLOR_BUFFER_BIT);
    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesForPatch]");
    //This todo still applies, especially since we needed to do a scissor thingy to
    //get this done correctly
    if(patch->isPartOfActiveSetWithNeighbours(activeSet)){//this should not not be the case
        //render the patch.

        std::shared_ptr<MeshPatchGpuHandle> patchGpu = patch->gpu.lock();

        if(patchGpu == nullptr){
            cout << "[MapInformationRenderer::renderTriangleReferencesForPatch]"
                    " This should not happen when this is invoked for a patch"
                    " which is part of an active set" << endl;
            assert(0);
            return;
        }

        size_t slot = patchGpu->triangles->getStartingIndex();
        size_t size = patchGpu->triangles->getSize();
        //it does not seem like we are running out of empty elements
        //cout << "DEBUG: " << m_map->m_gpuGeomStorage.triangleBuffer->getFreeElements() << endl;

        gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesForPatch]");
        glDrawArrays(GL_TRIANGLES,//GL_TRIANGLES,
                     slot*3,
                     size*3);


        gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesForPatch]");


        //Add the dependency of this central patch
        MeshTextureGpuHandle::Dependency dependency;
        dependency.trianglePositionOnGpu = patchGpu->triangles->getStartingIndex();
        dependency.trianglesVersion = patch->triangles_version;
        dependency.geometry = patch;
        gpuTex->refTexDependencies.push_back(dependency);

        patch->double_stitch_mutex.lock();
        for(size_t i=0;i<patch->double_stitches.size();i++){
            DoubleStitch &stitch = *patch->double_stitches[i].get();
            if(stitch.patches[0].lock()==patch){
                //only render when patch is main patch
                std::shared_ptr<TriangleBufConnector> stitchGpu = stitch.triangles_gpu.lock();
                if(stitchGpu==nullptr){
                    cout << "[MapInformationRenderer::renderTriangleReferencesForPatch]"
                            " This should not happen when this is invoked for a patch"
                            " which is part of an active set" << endl;
                    assert(0);
                    return;
                }

                slot = stitchGpu->getStartingIndex();
                size = stitchGpu->getSize();
                glDrawArrays(GL_TRIANGLES,//GL_TRIANGLES,
                             slot*3,
                             size*3);

                gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesForPatch]");
                //Add the dependency of this stitch
                dependency.trianglePositionOnGpu = stitchGpu->getStartingIndex();
                dependency.trianglesVersion = stitch.triangles_version;
                dependency.geometry = patch->double_stitches[i];
                gpuTex->refTexDependencies.push_back(dependency);
            }

        }
        patch->double_stitch_mutex.unlock();

        patch->triple_stitch_mutex.lock();

        for(size_t i=0;i<patch->triple_stitches.size();i++) {
            TripleStitch &stitch = *patch->triple_stitches[i].get();
            if(stitch.patches[0].lock()!=patch){
                continue;
            }
            std::shared_ptr<TriangleBufConnector> stitchGpu = stitch.triangles_gpu.lock();
            if(stitchGpu==nullptr){
                cout << "[MapInformationRenderer::renderTriangleReferencesForPatch]"
                        " This should not happen when this is invoked for a patch"
                        " which is part of an active set" << endl;
                assert(0);
                return;
            }
            slot = stitchGpu->getStartingIndex();
            size = stitchGpu->getSize();
            glDrawArrays(GL_TRIANGLES,//GL_TRIANGLES,
                         slot*3,
                         size*3);

            gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesForPatch]");

            //Add the dependency of this stitch
            dependency.trianglePositionOnGpu = stitchGpu->getStartingIndex();
            dependency.trianglesVersion = stitch.triangles_version;
            dependency.geometry = patch->triple_stitches[i];
            gpuTex->refTexDependencies.push_back(dependency);
        }

        patch->triple_stitch_mutex.unlock();


    }else{

        cout << "[MapInformationRenderer::renderTriangleReferencesForPatch] "
                "Can't render lookup texture since stitches and neighbouring "
                "points are not part of active set!" << endl;
        assert(0);
    }

    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesForPatch] After rendering");


    glDisable(GL_SCISSOR_TEST);

}

void MapInformationRenderer::renderTriangleReferencesAndDepth(ActiveSet *activeSet,
                                                              Matrix4f projection,
                                                              Matrix4f pose)
{
    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesAndDepth] Before rendering");

    triangleRefDepthProg->use();



    //after setting up the uniforms we set up all the possible
    glUniformMatrix4fv(0,1,false,(GLfloat*)&pose);
    glUniformMatrix4fv(1,1,false,(GLfloat*)&projection);

    gfx::GLUtils::checkForOpenGLError("[ RenderMapPresentation::render] Right at rendering patches");


   /* struct Uniforms{
        int maxNrTexPoints;
        int maxNrVertices;
    };
    Uniforms uniforms;
    uniforms.maxNrVertices=activeSet->gpuGeomStorage->m_maxNrVerticesPerPatch;
    uniforms.maxNrTexPoints= activeSet->gpuGeomStorage->m_maxNrTexCoordsPerPatch;
    glUniform1i(2,uniforms.maxNrTexPoints);
    glUniform1i(3,uniforms.maxNrVertices);
    */
    //the vertex buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
                     activeSet->gpuGeomStorage->vertexBuffer->getGlName());

    //bind texture coordinates
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
                     activeSet->gpuGeomStorage->texPosBuffer->getGlName());

    //the triangle buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
                     activeSet->gpuGeomStorage->triangleBuffer->getGlName());

    //the patch information
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
                     activeSet->gpuGeomStorage->patchInfoBuffer->getGlName());


    activeSet->drawEverything();
    gfx::GLUtils::checkForOpenGLError("[MapInformationRenderer::renderTriangleReferencesAndDepth] Right after issuing render commands.");

    glFinish();

}

void
MapInformationRenderer::render(ActiveSet *activeSet, Eigen::Matrix4f projection, Eigen::Matrix4f pose,
                               Mat *depth, Mat *normals, Mat *color, Mat *labels) {

    PerThread pt = perThreadGlObjects[this_thread::get_id()];
    //activate VBO
    glBindFramebuffer(GL_FRAMEBUFFER,pt.combinedFBO);

    gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::render] Before rendering");

    unifiedInfoProg->use();

    //TODO: bind the framebuffer!



    glClearColor(NAN,NAN,NAN,NAN);
    //glClearColor(1,1,0,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    //set one certain buffer to something.....
    int clear[] = {-1,-1,-1,-1};
    glClearBufferiv(GL_COLOR,2,clear);
    float clear2[] = {0,0,1.0f,0}; // try it with float debugwise
    glClearBufferfv(GL_COLOR,3,clear2);





    //todo: setup all the other stuff!!!




    Matrix4f _pose=pose.inverse();
    Matrix4f _projection=projection.inverse();//maybe needed for proper depth calculation

    glUniformMatrix4fv(0,1,false,(GLfloat*)&_pose);
    glUniformMatrix4fv(1,1,false,(GLfloat*)&projection);
    gfx::GLUtils::checkForOpenGLError("[ RenderMapPresentation::render] Right at rendering patches");


    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
                     activeSet->gpuGeomStorage->vertexBuffer->getGlName());

    //bind texture coordinates
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
                     activeSet->gpuGeomStorage->texPosBuffer->getGlName());

    //the triangle buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
                     activeSet->gpuGeomStorage->triangleBuffer->getGlName());

    //the patch information
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
                     activeSet->gpuGeomStorage->patchInfoBuffer->getGlName());



    gfx::GLUtils::checkForOpenGLError("[MapInformationRenderer::render] Right before issuing render commands.");

    activeSet->drawEverything();

    gfx::GLUtils::checkForOpenGLError("[MapInformationRenderer::render] Right after issuing render commands.");

    glFinish();
    pt.zTexture->downloadData(depth->data);
    pt.normalTexture->downloadData(normals->data);
    pt.colorTexture->downloadData(color->data);
    pt.labelTexture->downloadData(labels->data);
    glFinish();

}

Vector4f MapInformationRenderer::renderAndExtractInfo(Eigen::Matrix4f view,
                          Eigen::Matrix4f proj,
                          bool renderVisibleFromCam,
                          GLFWwindow *rootContext,
                          int width,int height,
                          int x, int y,
                          int* patchInd,
                          int* triangleInd){
    gfx::GLUtils::checkForOpenGLError("[MapInformationRenderer::renderInfo] Before rendering");

    m_map->activeSetUpdateMutex.lock();
    shared_ptr<ActiveSet> activeSet1 = m_map->activeSetUpdate;
    m_map->activeSetUpdateMutex.unlock();

    m_map->activeSetRenderingMutex.lock();
    shared_ptr<ActiveSet> activeSet2 = m_map->activeSetRendering;
    m_map->activeSetRenderingMutex.unlock();



    GLuint screenFBO;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING,(GLint*)&screenFBO);
    //create a texture and fbo (+ depth texture) we render into
    GLuint FBO;
    glGenFramebuffers(1,&FBO);
    glBindFramebuffer(GL_FRAMEBUFFER,FBO);


    gfx::GLUtils::checkForOpenGLError("[ScaleableMap::renderInfo] Setting up stuff");

    GLuint refTex;
    GLuint geomTex;
    glGenTextures(1, &refTex);
    glBindTexture(GL_TEXTURE_2D,refTex);
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA32F, width, height, 0,GL_RGBA, GL_FLOAT, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, refTex, 0);
    glGenTextures(1, &geomTex);
    glBindTexture(GL_TEXTURE_2D,geomTex);
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA32F, width, height, 0,GL_RGBA, GL_FLOAT, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, geomTex, 0);


    gfx::GLUtils::checkForOpenGLError("[ScaleableMap::renderInfo] Setting up stuff");


    GLuint depthBufferTex;
    glGenTextures(1,&depthBufferTex);
    glBindTexture(GL_TEXTURE_2D,depthBufferTex);
    glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT32,
                 width, height, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,depthBufferTex,0);


    gfx::GLUtils::checkForOpenGLError("[ScaleableMap::renderInfo] Setting up stuff");
    //initialize with NAN

    //http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/

    GLenum DrawBuffers[2] = {GL_COLOR_ATTACHMENT0,GL_COLOR_ATTACHMENT1};
    glDrawBuffers(2, DrawBuffers); // "1" is the size of DrawBuffers

    gfx::GLUtils::checkForOpenGLError("[ScaleableMap::renderInfo] Setting up stuff");

    glViewport(0,0,width,height);
    glClearColor(NAN,NAN,NAN,NAN);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    gfx::GLUtils::checkForOpenGLError("[ScaleableMap::renderInfo] Setting up stuff");

    if(renderVisibleFromCam){
        if(activeSet1!=nullptr){
            m_map->m_informationRenderer.renderTriangleReferencesAndDepth(activeSet1.get(),proj,view);
        }
        if(activeSet2!=nullptr){
            m_map->m_informationRenderer.renderTriangleReferencesAndDepth(activeSet2.get(),proj,view);
        }
    }
    //TODO: also render the low detail stuff... at least do it for the
    m_map->lowDetailRenderer.renderGeometry(proj,view);



    glFinish();

    cv::Mat cpuTriRef(height,width,CV_32FC4);
    cv::Mat cpuGeomRef(height,width,CV_32FC4);
    //download the data.
    glBindTexture(GL_TEXTURE_2D,refTex);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_FLOAT,cpuTriRef.data);
    glBindTexture(GL_TEXTURE_2D,geomTex);
    glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_FLOAT,cpuGeomRef.data);

    //cv::imshow("cpuTriRef",cpuTriRef);
    //cv::imshow("cpuGeomRef",cpuGeomRef);
    //cv::waitKey(1);



    //rebind the original screen framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER,screenFBO);



    //all of this is just a one time thingy:
    glDeleteTextures(1,&refTex);
    glDeleteTextures(1,&geomTex);
    glDeleteFramebuffers(1,&FBO);


    //now read out the required data.... because we need it

    cout << "clicked on pixel" <<
         x << " x " << y << endl;
    cout << "content: " << endl;
    Eigen::Vector4f point = cpuGeomRef.at<Eigen::Vector4f>(y,x);
    cv::Vec4i ref = cpuTriRef.at<cv::Vec4i>(y,x);
    cout << "point " << point << endl;
    if(ref[3]!=10.0f){
        cout << "patch index " <<  *((int*)&ref[0]) << endl;
        cout << "triangle index " <<  *((int*)&ref[1]) << endl;
    }else{
        cout << "not a valid triangle" << endl;
    }

    if(patchInd != nullptr){
        *patchInd = ref[0];
    }
    if(triangleInd != nullptr){
        *triangleInd = ref[1];
    }
    return point;

}