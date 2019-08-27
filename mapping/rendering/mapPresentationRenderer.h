#ifndef FILE_RENDER_MAP_PRESENTATION
#define FILE_RENDER_MAP_PRESENTATION

#include <gpuTex.h>
#include <shader.h>
#include <memory>

#include <GL/gl.h>

//TODO: rename this to something like presentationRenderer

class MeshReconstruction;
class ActiveSet;

class GLFWwindow;

class Worker;

class MapPresentationRenderer{ //TODO: maybe split this up
private:
    int m_width;
    int m_height;


    std::shared_ptr<gfx::GpuTex2D> m_debugTexture;

    GLuint m_geomDensityFBO;
    std::shared_ptr<gfx::GpuTex2D> m_geomDensityTexture;
    GLuint m_indexFBO;
    std::shared_ptr<gfx::GpuTex2D> m_indexTexture;

    //the one and only depth buffer shared for all the information rendering tasks... (hopefully)
    /*
    GLuint m_depthBufferTex;

    GLuint m_depthFBO;
    std::shared_ptr<gfx::GpuTex2D> m_depthTexture;
    GLuint m_depthVAO;
    */
    GLuint m_VAO;
    static std::weak_ptr<gfx::GLSLProgram> s_rgbProgram;
    std::shared_ptr<gfx::GLSLProgram> m_rgbProgram;


    Worker *renderingActiveSetUpdateWorker=nullptr;
    GLuint debugVAO;
    std::shared_ptr<gfx::GLSLProgram> debugProgram;

   MeshReconstruction* m_map;
public:
    MapPresentationRenderer(int width=640,int height=480);
    ~MapPresentationRenderer();
    void initInContext(int width,int height,MeshReconstruction* map);
    void initInContext();

    // render an additional wireframe
    bool showWireframe=true;

    // all the other render modes.
    bool renderPatchIds;
    int colorMode = 0;
    int shadingMode = 0;

    //void render(Eigen::Matrix4f projection, Eigen::Matrix4f pose);


    void render(ActiveSet *activeSet, Eigen::Matrix4f projection, Eigen::Matrix4f pose);



    void renderInWindow(Eigen::Matrix4f view, Eigen::Matrix4f proj,bool renderVisibleFromCam,GLFWwindow* rootContext);



};

#endif
