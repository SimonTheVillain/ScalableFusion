#include "debugRender.h"
#include "../gfx/glUtils.h"
#include "base/meshStructure.h"


const std::string debug_frag =
#include "debugGeometry.frag"
;

const std::string debug_vert =
#include "datastructure.glsl"
#include "debugGeometry.vert"
;


RenderDebugInfo* thatOneDebugRenderingThingy;

using namespace std;
using namespace gfx;
using namespace Eigen;

RenderDebugInfo::RenderDebugInfo() {
    shader = make_shared<GLSLProgram>();
    shader->compileShader(debug_frag,
                          GLSLShader::GLSLShaderType::FRAGMENT,
                          "debugGeometry.frag");
    shader->compileShader(debug_vert,
                          GLSLShader::GLSLShaderType::VERTEX,
                          "debugGeometry.vert");
    shader->link();

}

RenderDebugInfo::~RenderDebugInfo() {

}

void RenderDebugInfo::render(Eigen::Matrix4f proj,
                            Eigen::Matrix4f _camPose) {


    /*
    if(count == 0){
        return;
    }
     */

    if(renderedPatch == nullptr){
        //return;
    }
    if(patches.size()==0){
        return;
    }
    shader->use();

    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding triangleBuffer");
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,vertexBuffer);
                     //activeSet->gpuGeomStorage->vertexBuffer->getGlName());


    //bind texture coordinates
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,texPosBuffer);
                     //activeSet->gpuGeomStorage->texPosBuffer->getGlName());
    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding texPosBuffer");

    //the triangle buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,triangleBuffer);
                     //activeSet->gpuGeomStorage->triangleBuffer->getGlName());

    //the patch information
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,infoBuffer);
                     //activeSet->gpuGeomStorage->patchInfoBuffer->getGlName());


    glUniformMatrix4fv(0,1,false,(GLfloat*)&_camPose);
    glUniformMatrix4fv(1,1,false,(GLfloat*)&proj);


    mutex.lock();
    /*
    if(count == 0){
        return;
    }
    */
    /*
    glDrawArrays(GL_TRIANGLES,
                 startIndex*3,
                 count*3);

    glDrawArrays(GL_POINTS,
                 startIndex*3,
                 count*3);

    */

    for(int i = 0;i<patches.size();i++){
        MeshPatch* p = patches[i].patch;
        glUniform4f(2,patches[i].r,patches[i].g,patches[i].b,1);
        glUniform1i(3,-1);
        if(p != nullptr){
            int start = p->gpu.lock()->triangles->getStartingIndex();
            int nr = p->gpu.lock()->triangles->getSize();
            if(i!=0 && forceDstGeom){
                glUniform1i(3,p->gpu.lock()->verticesDest->getStartingIndex());
            }
            glDrawArrays(GL_TRIANGLES,
                         start*3,
                         nr*3);

            glDrawArrays(GL_POINTS,
                         start*3,
                         nr*3);


            if(i!=0){
                continue;
            }
            for(shared_ptr<DoubleStitch> stitch : p->doubleStitches){
                shared_ptr<TriangleBufConnector> triangles = stitch->trianglesGpu.lock();
                if(triangles == nullptr){
                    continue;//doesn't need to be
                }
                start = triangles->getStartingIndex();
                nr = triangles->getSize();
                glDrawArrays(GL_TRIANGLES,
                             start*3,
                             nr*3);

                glDrawArrays(GL_POINTS,
                             start*3,
                             nr*3);
                gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding texPosBuffer");
            }


            for(shared_ptr<TripleStitch> stitch : p->tripleStitches){
                shared_ptr<TriangleBufConnector> triangles = stitch->trianglesGpu.lock();
                if(triangles == nullptr){
                    continue;//doesn't need to be
                }
                start = triangles->getStartingIndex();
                nr = triangles->getSize();
                glDrawArrays(GL_TRIANGLES,
                             start*3,
                             nr*3);

                glDrawArrays(GL_POINTS,
                             start*3,
                             nr*3);
                gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding texPosBuffer");
            }

        }
    }

    gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding texPosBuffer");
    mutex.unlock();

    glFinish();
}

void RenderDebugInfo::setIndexCount(int startVertex, int vertexCount) {
    mutex.lock();
    this->startIndex = startVertex;
    this->count = vertexCount;
    mutex.unlock();

}

void RenderDebugInfo::setPatch(MeshPatch *patch) {
    renderedPatch = patch;
}

void RenderDebugInfo::addPatch(MeshPatch *patch, float r, float g, float b) {
    ShowPatch task;
    task.patch = patch;
    task.r = r;
    task.g = g;
    task.b = b;
    task.a = 1;
    patches.push_back(task);

}
