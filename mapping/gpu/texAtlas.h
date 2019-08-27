#ifndef FILE_TEX_ATLAS_H
#define FILE_TEX_ATLAS_H

#include <memory>
#include <vector>
#include <gpuTex.h>
#include <mutex>
#include <stack>
#include <thread>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "../cuda/gpuMeshStructure.h"
#include "threadSafeFBO_VAO.h"



class TexAtlas;
class TexAtlasTex;
class TexAtlasPatch;
class GarbageCollector;




class TexAtlas{
private:
    std::mutex mutex;

    //vector of vector with sizes
    struct SafeVector{
        std::mutex texMutex;
        std::vector<std::weak_ptr<TexAtlasTex>> tex;
    };

    SafeVector *textures;
    GLuint intType;
    GLuint format;
    GLuint type;
    int cvType;
    int maxRes;
    ThreadSafeFBOStorage* fboStorage;
    GarbageCollector * garbageCollector;

#ifdef GL_MEMORY_LEAK_WORKAROUND
    std::vector<std::shared_ptr<TexAtlasTex>> texRetainer;
#endif
public:
    //probably don't need cv type
    TexAtlas(GarbageCollector* garbageCollector,GLuint intType,GLuint type, GLuint format,int cvType,int res=1024,
             ThreadSafeFBOStorage* fbos=nullptr);//create one texture atlas for every type of
    ~TexAtlas();
    std::shared_ptr<TexAtlasPatch> getTexAtlasPatch(cv::Size2i size);
    size_t getMaxRes(){return maxRes;}

    int getCvType(){return cvType;}
    GLuint getGlIntType(){return intType;}
    GLuint getGlType(){return type;}


    int countPatches();
    int countTex();

};


class TexAtlasTex{
    friend TexAtlasPatch;
    friend TexAtlas;
private:
    std::shared_ptr<gfx::GpuTex2D> tex;
    int tileSize;//8,16,32,64,etc.
    int cvType;
    std::mutex occupantsMutex;
    //std::vector<std::weak_ptr<TexAtlasPatch>> occupants;//TODO: remove this!!!!!
    cv::Rect2i posFromIndex(int i);

    //TODO: let this throw an exception if there is no slot left
    std::shared_ptr<TexAtlasPatch> getFreePatch(std::shared_ptr<TexAtlasTex> self);

    //TODO: this is genious... the stack tells us if and where there is place for new textures
    //all of this while being superfast. (has to be filled at constructor with empty slots)
    //TODO: also implement a system like this for the GPU GEOMETRY STORAGE)
    std::stack<int> freeSlots;
    void freeTexSlot(int inSlot);
    //TODO: make the constructor sort of private
    TexAtlasTex(GarbageCollector* garbageCollector,GLuint intType,GLuint type,
                GLuint format,int cvType, int res,int fullRes,
                ThreadSafeFBOStorage* fboStorage);

    //GLuint FBO=0;
    ThreadSafeFBO* fbo=nullptr;
    bool debug=false;

    std::thread::id debugThreadIdTexCreatedIn;
public:
    ~TexAtlasTex();
    std::mutex mutex;
    bool hasSlot();


    void showImage(std::string text);
    void showImage(std::string text,cv::Rect2i cutOut);

    GLuint getFBO();

    std::shared_ptr<gfx::GpuTex2D> getTex2D(){
        return tex;
    }
    int getCvType(){
        return cvType;
    }

    int countPatches();




};

class TexAtlasPatch{
    friend TexAtlasTex;
    friend TexAtlas;
private:
    std::shared_ptr<TexAtlasTex> tex;
    cv::Rect2i pos;//position and reserved size within the texture atlas
    cv::Size2i size;//size which is actually used
    int indexInAtlasTex=-1;

    TexAtlasPatch(std::shared_ptr<TexAtlasTex> &tex,cv::Rect2i &pos,int index);
public:
    ~TexAtlasPatch();
    std::shared_ptr<gfx::GpuTex2D> getTex();
    cv::Rect2i getPosition();

    cv::Rect2i getRect(){
        return cv::Rect(pos.x,pos.y,size.width,size.height);
    }

    std::shared_ptr<TexAtlasTex> getAtlasTex(){
        return tex;
    }


    int getFramebuffer();
    void setFramebufferActive();

    GLuint getFBO();
    void setViewport();
    void setViewport(cv::Size2i size);

    bool downloadData(void* data,cv::Rect2i rect);
    bool downloadData(void* data,cv::Size2i size){
        return downloadData(data,cv::Rect2i(pos.x,pos.y,size.width,size.height));
    }

    bool downloadAllData(void* data){
        return downloadData(data,pos);
    }
    bool downloadData(void* data){
        return downloadData(data,getRect());
    }

    bool uploadData(void* data,cv::Rect2i rect);
    bool uploadData(void* data,cv::Size2i size){
        return uploadData(data,cv::Rect2i(pos.x,pos.y,size.width,size.height));
    }

    bool uploadAllData(void* data){
        return uploadData(data,pos);
    }
    bool uploadData(void* data){
        return uploadData(data,getRect());
    }

    cudaSurfaceObject_t getCudaSurfaceObject();
    cudaTextureObject_t getCudaTextureObject();
    uint64_t getGlHandle();


    void showImage(std::string text);

    void showImage(std::string text,cv::Size2i size);

    GpuTextureInfo genTexInfo(cv::Size2i size, int texCoordStartingIndex);


    GpuTextureInfo genTexInfo(int texCoordStartingIndex);
    //test if this patch is required to
    bool isGpuResidencyRequired();
};

#endif
