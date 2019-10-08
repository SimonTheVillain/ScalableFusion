#ifndef FILE_GPUTEX_H
#define FILE_GPUTEX_H

#include <GL/glew.h>
//#include <cublas.h>
#include <cuda_gl_interop.h>

#include <string>
#include <vector>
#include <unordered_map>
#include <thread>
#include <mutex>



class GarbageCollector;



//TODO: incorporate cuda streams into the whole mess!!!!
//TODO: create a few constants for the constructor to
//easily initialize the most common texture types
namespace gfx{
class GpuTex2D{
private:
    uint64_t glHandle;
    GLuint glName;
    GLuint m_glInternalFormat;
    GLuint m_glType;
    GLuint m_glFormat;
    int channelCount;
    int byteCount;
    struct cudaGraphicsResource* cudaTextureResource;
    cudaResourceDesc cudaResDesc;
    cudaTextureObject_t cudaTextureReference;
    cudaSurfaceObject_t cudaSurfaceReference;
    cudaArray_t cudaArrayReference;
    int m_width,m_height;

    GarbageCollector* garbageCollector;
    std::mutex tokenMutex;
    std::unordered_map<std::thread::id,std::shared_ptr<bool>> residentToken;


    static int overallTexCount;
    static std::mutex overallTexListMutex;
    static std::vector<GpuTex2D*> overallTexList;
public:
    static int getTexCount();
    static std::vector<GpuTex2D*> getTexList();
    std::string name;

    //GarbageCollector *garbageCollector,//TODO: make the garbage collector mandatory
    GpuTex2D(GarbageCollector *garbageCollector,GLuint glInternalFormat,GLuint glFormat,GLuint glType,int width,int height,bool cudaNormalizedTexCoords,
            void* data=NULL,GLint filterType = GL_LINEAR);


    ~GpuTex2D();


    void uploadData(void* data); //if size==0 we only load
    void uploadData(void* data,int m_width,int m_height);
    void uploadData(void* data,int x,int y,int width,int height);

    void downloadData(void* data);
    void downloadData(void* data,int x, int y,int width, int height);

    GLuint getGlName(){return glName;}

    void makeResidentInThisThread();
    uint64_t getGlHandle(){return glHandle;}

    cudaTextureObject_t getCudaTextureObject(){return cudaTextureReference;}
    cudaSurfaceObject_t getCudaSurfaceObject(){return cudaSurfaceReference;}

    int getWidth(){return m_width;}

    int getHeight(){return m_height;}

    GLuint getGlInternalFormat(){return m_glInternalFormat;};
    GLuint getGlType(){return m_glType;};
    GLuint getGlFormat(){return m_glFormat;};


    //GLuint GetFbo();




};
}

#endif
