#ifndef FILE_RENDER_MAP_INFORMATIONS
#define FILE_RENDER_MAP_INFORMATIONS

//TODO: rename this to InformationRenderer (better Name, as i think)

//#include "scaleableMap.h"
#include <gpuTex.h>
#include <shader.h>
#include <memory>

#include <map>
#include <mutex>
#include <thread>

#include <GL/gl.h>

#include <opencv2/core.hpp>

//class SharedImage;
class MeshReconstruction;
class MeshPatch;
class MeshTexture;
class ActiveSet;

class GLFWwindow;
/**
 * @brief The RenderMapInformations class
 * does some of the rendering of the map to the current view (current camera pose).
 * Rendered information should include:
 * 1) depth: only new depth measurements that are similar to the current map get incorporated inteo
 *    the existing geometry. If new depth measurements are bigger than the projected map depth,
 *    we could should remove geometry obviously.
 *    If the new geometry is closer, it should be added to the map.
 *    If the new geometry is within range of the existing one.
 * 2) The geometry density. (low density means we can use the rendered indices to change geometry)
 *    High density in geometry means that in the index images for vertices does not show all the geometry.
 *    The geometry itself has to be iterated to see if it is in range of the depth buffer and see if it
 *    is fit to be altered trough the new data.
 * 3) The indices of the vertices (triangles have 3 values for the 3 vertices it is made out of).
 * 4) Borders of existing geometry.
 * 5) All of that magic we need for ICP or Dense Visual Odometry
 *
 * To do this, it mostly manages a bunch of shaders and buffers.
 * MAYBE LAYERED RENDERING MIGHT BE USEFUL WHEN NEEDING MULTIPLE IMAGES (only performance stuff)
 * or this: (who knows!)
 * http://stackoverflow.com/questions/31768783/get-element-id-in-vertex-shader-in-opengl
 */
class MapInformationRenderer{ //TODO: maybe split this up
    friend MeshReconstruction;
private:
    int m_width;
    int m_height;

public://TODO: remove public, just for debug purposes
    std::shared_ptr<gfx::GpuTex2D> m_debugTexture;
private:

    struct PerThread{
       GLuint indexFBO;
       std::shared_ptr<gfx::GpuTex2D> indexTexture;


       GLuint depthVAO=0;
       GLuint depthFBO=0;
       GLuint depthBufferTex;
       std::shared_ptr<gfx::GpuTex2D> depthTexture;


       std::shared_ptr<gfx::GpuTex2D> stdTexture;



        //TODO: these are the new textures

       GLuint combinedVAO=0;
       GLuint combinedFBO=0;
       std::shared_ptr<gfx::GpuTex2D> zTexture;
       std::shared_ptr<gfx::GpuTex2D> colorTexture;
       std::shared_ptr<gfx::GpuTex2D> labelTexture;
       std::shared_ptr<gfx::GpuTex2D> normalTexture;

    };
    //maybe thread_local variable
    std::map<std::thread::id,PerThread> perThreadGlObjects;

    //GLuint m_geomDensityFBO;
    //std::shared_ptr<gfx::GpuTex2D> m_geomDensityTexture;
    //GLuint m_indexFBO;
    //std::shared_ptr<gfx::GpuTex2D> m_indexTexture;



    //the one and only depth buffer shared for all the information rendering tasks... (hopefully)
    //GLuint m_depthBufferTex;

    //GLuint m_depthFBO;
   // std::shared_ptr<gfx::GpuTex2D> m_depthTexture;


    //std::mutex VAOMutex;//TODO: this, create a std::map that connects VAOs to mutexes
    //(i fear same has to be done for FBOs, which is a shame)
    //https://www.opengl.org/discussion_boards/showthread.php/169733-Sharing-FBOs-among-contexts
    //GLuint m_depthVAO;


    static std::mutex shaderMutex;
    static std::weak_ptr<gfx::GLSLProgram> s_depthProgram;
public://debug
    std::shared_ptr<gfx::GLSLProgram> m_depthProgram;
private:
   MeshReconstruction* m_map;


   static std::weak_ptr<gfx::GLSLProgram> s_triangleReferenceProgram;
   std::shared_ptr<gfx::GLSLProgram> m_triangleReferenceProgram;



   static std::weak_ptr<gfx::GLSLProgram> s_triangleRefDepthProgram;
   std::shared_ptr<gfx::GLSLProgram> triangleRefDepthProg;



   //combined rendering of labels,depth and normals. (TODO)
   std::shared_ptr<gfx::GLSLProgram> unifiedInfoProg;



public:
    MapInformationRenderer(int width=640,int height=480);
    ~MapInformationRenderer();
    void initInContext(int width,int height,MeshReconstruction* map);
    void initInContext();

    std::shared_ptr<gfx::GpuTex2D> getDepthTexture();

    std::shared_ptr<gfx::GpuTex2D> getStdTexture();


    void renderDepth(ActiveSet *activeSet, Eigen::Matrix4f projection, Eigen::Matrix4f pose);

    ///TODO: create a renderer for which patch is used in which image!!!
    //L.G. Debugging tool

    void bindRenderTriangleReferenceProgram();
    void renderTriangleReferencesForPatch(ActiveSet *activeSet, std::shared_ptr<MeshPatch> &patch,
                                          std::shared_ptr <MeshTexture> &targetTexture);



    void renderTriangleReferencesAndDepth(ActiveSet* activeSet,Eigen::Matrix4f projection,Eigen::Matrix4f pose);


    void render(ActiveSet *activeSet, Eigen::Matrix4f projection, Eigen::Matrix4f pose,
                cv::Mat *depth = nullptr, cv::Mat *normals = nullptr, cv::Mat *color = nullptr,
                cv::Mat *labels = nullptr);



    Eigen::Vector4f renderAndExtractInfo(Eigen::Matrix4f view,
                              Eigen::Matrix4f proj,
                              bool renderVisibleFromCam,
                              GLFWwindow *rootContext,
                              int width,int height,
                              int x, int y,
                              int* patchInd = nullptr,
                              int* triangleInd = nullptr);


};

#endif
