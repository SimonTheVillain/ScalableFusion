#ifndef DEBUG_RENDER_H
#define DEBUG_RENDER_H

#include "../gfx/shader.h"
#include <mutex>
#include <memory>


class MeshPatch;



class RenderDebugInfo{
private:
    std::mutex mutex;
    GLuint startIndex;

    GLuint count=0;
    std::shared_ptr<gfx::GLSLProgram> shader;
public:
    struct ShowPatch{
        float r,g,b,a;
        MeshPatch* patch;
    };
    std::vector<ShowPatch> patches;
    GLuint vertexBuffer;
    GLuint triangleBuffer;
    GLuint infoBuffer;
    GLuint texPosBuffer;

    bool forceDstGeom = false;

    MeshPatch* renderedPatch = nullptr;



    //also make the color configurable;


    RenderDebugInfo();

    ~RenderDebugInfo();

    void render(Eigen::Matrix4f proj,
                Eigen::Matrix4f _camPose);
    void setIndexCount(int index,int count);

    void addPatch(MeshPatch* patch, float r,float g,float b);
    void setPatch(MeshPatch* patch);



};

extern RenderDebugInfo* thatOneDebugRenderingThingy;

#endif