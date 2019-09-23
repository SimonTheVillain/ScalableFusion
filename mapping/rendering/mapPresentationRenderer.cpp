#include "mapPresentationRenderer.h"

#include "../meshReconstruction.h"
#include "../worker.h"
#include "mapInformationRenderer.h"
#include <GarbageCollector.h>

#include <opencv2/opencv.hpp>


//c++ 11 string literals as measure to include shader files
//http://stackoverflow.com/questions/20443560/how-to-practically-ship-glsl-shaders-with-your-c-software

#include <glUtils.h>
#include "ActiveSet.h"

#include "gpuErrchk.h"
const std::string presentation_frag =
#include "presentation.frag"
;
const std::string presentation_vert =
#include "datastructure.glsl"
#include "presentation.vert"
;



const std::string presentationDebug_frag =
#include "presentationDebug.frag"
;
const std::string presentationDebug_vert =
#include "presentationDebug.vert"
;


using namespace gfx;
using namespace std;
using namespace Eigen;


std::weak_ptr<gfx::GLSLProgram> MapPresentationRenderer::s_rgbProgram;

MapPresentationRenderer::MapPresentationRenderer(int width, int height)
{

}

MapPresentationRenderer::~MapPresentationRenderer()
{

}

void MapPresentationRenderer::initInContext(int width, int height, MeshReconstruction *map)
{

    m_map = map;
    m_width=width;
    m_height=height;
    initInContext();
}


void MapPresentationRenderer::initInContext()
{
    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::initInContext]");

    //render depth shader
    if(MapPresentationRenderer::s_rgbProgram.use_count()){//if the shader exists already copy the reference
        m_rgbProgram = MapPresentationRenderer::s_rgbProgram.lock();

    }else{//otherwise create a new shader

        m_rgbProgram = std::shared_ptr<GLSLProgram>(new GLSLProgram());
        m_rgbProgram->compileShader( presentation_frag,
                                 gfx::GLSLShader::GLSLShaderType::FRAGMENT,
                                 "presentation.frag" );
        m_rgbProgram->compileShader( presentation_vert,
                                 gfx::GLSLShader::GLSLShaderType::VERTEX,
                                 "presentation.vert" );
        m_rgbProgram->link();
        MapPresentationRenderer::s_rgbProgram = m_rgbProgram;
    }
    glGenVertexArrays(1,&m_VAO);
    glBindVertexArray(m_VAO);





    debugProgram = std::shared_ptr<GLSLProgram>(new GLSLProgram());
    debugProgram->compileShader( presentationDebug_frag,
                             gfx::GLSLShader::GLSLShaderType::FRAGMENT,
                             "presentationDebug.frag" );
    debugProgram->compileShader( presentationDebug_vert,
                             gfx::GLSLShader::GLSLShaderType::VERTEX,
                             "presentationDebug.vert" );
    debugProgram->link();
    glGenVertexArrays(1,&debugVAO);
    glBindVertexArray(debugVAO);




}

void MapPresentationRenderer::render(ActiveSet *activeSet,Eigen::Matrix4f projection, Eigen::Matrix4f pose)
{
    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] before doing anything.");

    if(activeSet==nullptr){
        return;
    }
    if(activeSet->retained_double_stitches.size() == 0){
        return;
    }






    //end of debug

    //cout << "started rendering the active set" << endl;

    m_rgbProgram->use();

    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding shader.");
    glBindVertexArray(m_VAO);

    //after setting up the uniforms we set up all the possible
    glUniformMatrix4fv(0,1,false,(GLfloat*)&pose);
    glUniformMatrix4fv(1,1,false,(GLfloat*)&projection);

    glUniform1i(4,(int)showWireframe);
    glUniform1i(5,(int)colorMode);
    glUniform1i(6,(int)shadingMode);


    /*
    struct Uniforms{
        int maxNrTexPoints;
        int maxNrVertices;
    };
    Uniforms uniforms;
    uniforms.maxNrVertices=activeSet->gpuGeomStorage->m_maxNrVerticesPerPatch;
    uniforms.maxNrTexPoints= activeSet->gpuGeomStorage->m_maxNrTexCoordsPerPatch;

    //glUniform1i(8,maxNrTriangles); // might be that this is not needed anymore
                                    //might also be that we need much more than that ( see texture coordinates)
    //glUniform2i(8,uniforms.maxNrTriangles,uniforms.maxNrTexPoints);
    //glUniform1i(2,uniforms.maxNrTexPoints);//debug: womehow this works but the line above doesn't
    glUniform1i(2,uniforms.maxNrTexPoints);
    glUniform1i(3,uniforms.maxNrVertices);
    */
    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Setting up uniforms.");
    //the vertex buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
                     activeSet->gpu_geom_storage->vertexBuffer->getGlName());


    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding vertexBuffer");
    //bind texture coordinates
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
                     activeSet->gpu_geom_storage->texPosBuffer->getGlName());
    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding texPosBuffer");

    //the triangle buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
                     activeSet->gpu_geom_storage->triangleBuffer->getGlName());
    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding triangleBuffer");

    //the patch information
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
                     activeSet->gpu_geom_storage->patchInfoBuffer->getGlName());


    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding buffers");


    //TODO: this is baaaad, how to get rid of this lock?
    //this blocks everything.
    //somehow we should prevent this from locking the whole list while rendering
    //if we mutex every single patch while rendering?
    //    -> impossible, the tex updates come all at once. and would be blocked
    // we need a non blocking memory system without stalling everything
    // one problem still is that the gpu memory could be changing while the
    //objects are being rendered.
    //mutexing all objects at once is bad tough
    //WAIT AND SEE

    //this is not too bad tough since rendering takes only 15ms at most (on the old gpu)
    //on the new one its way better.
    //activeSet->vectorUpdateMutex.lock();


    glUniform4f(7,0,0,0,1);
    activeSet->drawPatches();

    glUniform4f(7,0.7f,0,0,1);
    activeSet->drawDoubleStitches();

    glUniform4f(7,0,0,0.7f,1);
    activeSet->drawTripleStitches();



    glFinish();//i know this isn't ideal for performance but we need to make sure that the data usage is
    // contained within this function so it won't be modified outside

    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] After doing everything.");



}
void MapPresentationRenderer::renderInWindow(Eigen::Matrix4f view, Eigen::Matrix4f proj,bool renderVisibleFromCam,GLFWwindow* rootContext){
    //return;//TODO: remove this debug measure
    //disgusting debug attempt
    bool disableRenderOfUserCamera=false; //false means we render stuff seen by the user camera

    //get the visible ones
    if(renderVisibleFromCam && !disableRenderOfUserCamera){
        Vector4f intrinsics(700,700,320,240); //to see more of everything use
        Vector2f res(640,480);
        //since in opengl the camera faces towards -z we have to flip it
        //for our visibility thingy
        Matrix4f flipz = Matrix4f::Identity();
        flipz(2,2)=-1;
        Matrix4f camPose = (flipz*view).inverse();

        //we are doing this in a secondary thread
        //lets create a worker thread if there is none
        if(renderingActiveSetUpdateWorker == nullptr &&
           !disableRenderOfUserCamera){//DEBUG
            GLFWwindow** context = new GLFWwindow*;
            auto initializer = [&](GLFWwindow* parentContext,GLFWwindow** newContext){
                //init opengl context here
                glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
                glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
                glfwWindowHint(GLFW_VISIBLE, 0);
                *newContext = glfwCreateWindow(640, 480,
                                               "HOPE U NO VISIBLE",
                                               nullptr, parentContext);
                glfwMakeContextCurrent(*newContext);
                glewExperimental = GL_TRUE;
                glewInit();
                glGetError();
                //TODO: it feels weird calling this method within this class
                m_map->m_informationRenderer.initInContext(640,480,m_map);
            };
            auto cleaner = [&](GLFWwindow** context){
                m_map->fboStorage.forceGarbageCollect();
                m_map->garbageCollector->forceCollect();
                glFinish();
                glfwDestroyWindow(*context);
                delete context;
            };
            function<void ()> initFunction = std::bind(initializer,rootContext,context);
            function<void ()> cleanupFunction = std::bind(cleaner,context);
            renderingActiveSetUpdateWorker =
                    new Worker(initFunction,"UpSetRend",cleanupFunction);//updateActiveSetForRendering
        }
        auto task = [&](Matrix4f invCamPose,
                        Vector4f intrinsics,
                        Vector2f res,
                        float viewDistance){
            vector<shared_ptr<MeshPatch>> visiblePatches =
                    m_map->octree.getObjects(invCamPose,
                                      intrinsics,
                                      res,
                                      viewDistance);


            cudaDeviceSynchronize();
            gpuErrchk( cudaPeekAtLastError() );
            shared_ptr<ActiveSet> activeSet =
                    m_map->m_gpuGeomStorage.makeActiveSet(visiblePatches,m_map);


            //cleanup VBO and VAOs that are deleted but only used within this thread
            m_map->cleanupGlStoragesThisThread();
            m_map->garbageCollector->collect();
            glFinish();
            cudaDeviceSynchronize();
            gpuErrchk( cudaPeekAtLastError() );
            //This might be obsolete when the makeActiveSet is changed to append all
            //the references
            //is this really needed?
            //activeSet->securePatchTextures();
            //store the active set
            m_map->activeSetRenderingMutex.lock();
            m_map->activeSetRendering = activeSet;
            m_map->activeSetRenderingMutex.unlock();

        };

        //the update worker is not queuing the update tasks.
        renderingActiveSetUpdateWorker->setNextTask(std::bind(task,
                                                              camPose,
                                                              intrinsics,
                                                              res,
                                                              1.0f));

    }else{

        m_map->activeSetRenderingMutex.lock();
        m_map->activeSetRendering = nullptr;
        m_map->activeSetRenderingMutex.unlock();
    }


    //at last we render the active set
    m_map->activeSetUpdateMutex.lock();
    shared_ptr<ActiveSet> activeSetWork = m_map->activeSetUpdate;
    m_map->activeSetUpdateMutex.unlock();


    m_map->activeSetRenderingMutex.lock();
    shared_ptr<ActiveSet> activeSetDisplay;// = activeSetRendering;
    if(!disableRenderOfUserCamera){
        activeSetDisplay = m_map->activeSetRendering;
    }
    m_map->activeSetRenderingMutex.unlock();
    /*
    if(!glIsTextureHandleResidentARB(m_gpuGeomStorage.deleteDebugTexReference)){
        glMakeTextureHandleResidentARB(m_gpuGeomStorage.deleteDebugTexReference);
    }*/


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );


    //set of patches excluded for the low detail render
    std::vector<shared_ptr<ActiveSet>> sets;
    if(renderVisibleFromCam){
        render(activeSetWork.get(),proj,view);
        sets.push_back(activeSetWork);
    }


    if(activeSetDisplay!=nullptr && renderVisibleFromCam){
        //store it in the list to exlude it from the low detail render
        sets.push_back(activeSetDisplay);
        //also render it
        render(activeSetDisplay.get(),proj,view);
    }


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    m_map->lowDetailRenderer.renderExceptForActiveSets(sets,
                                                proj,view);
}