#ifndef FILE_GPU_BUFFER_H
#define FILE_GPU_BUFFER_H

#include <thread>
#include <mutex>
#include <vector>
#include <set>
#include <map>
#include <utility>


#include <GL/glew.h>

#include <cuda.h>
#include <cuda_gl_interop.h>

#include "../cuda/gpuMeshStructure.h"



template<typename T>
class GpuBuffer;

//TODO: get rid of this??????? ( i don't understand the need for this anymore
class GpuBufferConnectorBase{
/* this only exists so we can store
 * shared_ptr to this without it being deleted unintentionally!
 * */
};

template<typename T>

//TODO: rename BufferConnector to bufferHandle / slot? (except for when it really is more a connector than a slot
class GpuBufferConnector{// : public GpuBufferConnectorBase{
    friend GpuBuffer<T>;
private:
    uint32_t index;//the starting index for this slot
    uint32_t size;//size

    GpuBuffer<T>* buffer=0;

    GpuBufferConnector(GpuBuffer<T>* buffer,uint32_t index, uint32_t size);
public:


    ~GpuBufferConnector();

    uint32_t getStartingIndex(){return index;}

    T* getStartingPtr(){return &(buffer->getCudaPtr()[index]);}


    void upload(T *data,uint32_t offset, uint32_t count);
    void download(T* data,uint32_t offset, uint32_t count);


    void upload(T* data){
        upload(data,0,size);
    }

    void download(T* data){
        download(data,0,size);
    }
    uint32_t getSize(){return size;}


};

//TODO: rename this to GpuBufferCollection
template<typename T>
class GpuBuffer{
private:
    std::mutex mutex;

    //store the free slots of a certain size
    //first its only one, of full size, smaller ones get split off of this one
    //the free slot gets moved down the according size
    //problems:
    //1) giving back one element and connecting this whith existing slots
    //probably doable with start and end map!
    //2) finding the next best slot (next biggest key)
    // maps are sorted
    // https://stackoverflow.com/questions/1660195/c-how-to-find-the-biggest-key-in-a-stdmap


    //key: size, value: adress (maybe replace this vector with a set(sorted))
    std::map<size_t,std::set<uint32_t>> freeSlots;

    //key: starting index, value: size
    std::map<uint32_t,uint32_t> starts;

    //key: ending index, value: size
    std::map<uint32_t,uint32_t> ends;

    GLuint glBuffer;
    cudaGraphicsResource_t cudaGraphicsResource;
    T* cudaResourcePtr;
    bool debug=false;
    size_t debugAbsoluteSize;//debug because it is unused
public:
    GpuBuffer(size_t elementCount,GLuint bufferType = GL_ARRAY_BUFFER,bool debugNoGpu=false);
    ~GpuBuffer();


    std::shared_ptr<GpuBufferConnector<T>> getBlock(size_t elementCount);

    void rejoinBlock(GpuBufferConnector<T> *block, bool firstLevel = true);


    void upload(T* data, uint32_t index, uint32_t count);
    void download(T* data,uint32_t index, uint32_t count);
    GLuint getGlName(){return glBuffer;}
    T* getCudaPtr(){return cudaResourcePtr;}


    uint32_t getUsedElements();
    uint32_t getFreeElements();
};

typedef GpuBuffer<GpuVertex> VertexBuffer;
typedef GpuBuffer<Eigen::Vector2f> TexCoordBuffer;
typedef GpuBuffer<GpuTriangle> TriangleBuffer;
typedef GpuBuffer<GpuPatchInfo> PatchInfoBuffer;
typedef GpuBuffer<GLint> IndexToPatchInfoBuffer;

typedef GpuBufferConnector<GpuVertex> VertexBufConnector;
typedef GpuBufferConnector<Eigen::Vector2f> TexCoordBufConnector;
typedef GpuBufferConnector<GpuTriangle> TriangleBufConnector;//TODO: see if this checks out... otherwise go back to "typedef"
typedef GpuBufferConnector<GpuPatchInfo> PatchInfoBufConnector;

void testBuffer();

#endif
