#include "gpuBuffer.h"
#include <iostream>
#include <assert.h>

#include <cuda_gl_interop.h>

#include "../cuda/gpuErrchk.h"


using namespace std;

template<typename T>
GpuBuffer<T>::GpuBuffer(size_t elementCount,GLuint bufferType, bool debugNoGpu)
{
    this->debugAbsoluteSize=elementCount;
    debug = debugNoGpu;
    if(!debugNoGpu){
    //create opengl buffer and connect it to cuda:


    glGenBuffers(1,&glBuffer);
    glBindBuffer(bufferType,glBuffer);
    glBufferData(bufferType,sizeof(T)*elementCount,0,GL_DYNAMIC_DRAW);

    cudaError_t error_test;
    error_test = cudaGraphicsGLRegisterBuffer(&cudaGraphicsResource,glBuffer,
                                              cudaGraphicsMapFlagsNone);//TODO: test these
    gpuErrchk( error_test );
    error_test = cudaGraphicsMapResources(1,&cudaGraphicsResource);//stream 0
    gpuErrchk( error_test );
    size_t size;
    error_test = cudaGraphicsResourceGetMappedPointer((void**)&cudaResourcePtr,
                                                      &size,cudaGraphicsResource);
    gpuErrchk( error_test );
    error_test = cudaGraphicsUnmapResources(1,&cudaGraphicsResource);//stream0
    gpuErrchk( error_test );
    //create a list with occupied slots
    //m_texPosBufferSlotOccupants = new weak_ptr<MeshTexture>[m_maxNrLoadedTextures];

    error_test = cudaGraphicsMapResources(1,&cudaGraphicsResource);//stream 0
    gpuErrchk( error_test );

    }


    //fill the structures with one giant free block
    freeSlots[elementCount].insert(0);
    starts[0]=elementCount;
    ends[elementCount-1]=elementCount;



}


template<typename T>
GpuBuffer<T>::~GpuBuffer()
{
    cudaError_t error_test;

    error_test = cudaGraphicsUnmapResources(1,&cudaGraphicsResource);
    gpuErrchk( error_test );
    error_test = cudaGraphicsUnregisterResource(cudaGraphicsResource);
    gpuErrchk( error_test );
    glDeleteBuffers(1,&glBuffer);
}


template<typename T>
std::shared_ptr<GpuBufferConnector<T>> GpuBuffer<T>::getBlock(size_t elementCount)
{

    if(elementCount==0){
        cout << "hey maybe this is killing everything" << endl;
        std::shared_ptr<GpuBufferConnector<T>> buf(new GpuBufferConnector<T>(this,0,0));
        return buf;
        assert(0);
    }
    mutex.lock();

    //first check if there are blocks of the requested size
    std::map<size_t,std::set<uint32_t>>::iterator it =
        freeSlots.lower_bound(elementCount);

    if(it == freeSlots.end()){
        std::shared_ptr<GpuBufferConnector<T>> empty;
        mutex.unlock();
        std::cout << "Error: Ran out of GPU memory for geometry" << endl;
        assert(0);//No free elements. everything from here on will be pain
        return empty;
    }

    //list of free blocks of this size
    set<uint32_t> &listFreeOfSize =
            it->second;
    //size of these blocks
    uint32_t sizeOfFree = it->first;

    //pointer to the first and last element of the chosen block
    uint32_t first = *listFreeOfSize.begin();
    uint32_t last = first + sizeOfFree - 1;

    //index, the adress of the block within the memory
    uint32_t index =
        last - elementCount + 1;

    //delete the ending points from the lists:
    if(starts.find(first) == starts.end()){
        cout << "this should not happen, how did it happen then?" << endl;
        assert(0);
    }
    starts.erase(starts.find(first));
    ends.erase(ends.find(last));
    listFreeOfSize.erase(listFreeOfSize.begin());
    if(listFreeOfSize.empty()){
        //if the list of free slots of this size is empty we also delete
        //this element in the higher level map
        freeSlots.erase(it);
    }

    if(index != first){// if the block is not exactly the requested size

        //but also create new entries for the object we split apart
        // (if necessary) and we are not taking a full block
        uint32_t newSize=sizeOfFree-elementCount;
        starts[first] = newSize;
        ends[first+newSize-1] = newSize;
        freeSlots[newSize].insert(first);
    }



    std::shared_ptr<GpuBufferConnector<T>>
            buffer(new GpuBufferConnector<T>(this,index,elementCount));
    mutex.unlock();
    return buffer;
}

//HEY! THIS SHOUTS FOR RECURSION!!!!!!

template<typename T>
void GpuBuffer<T>::rejoinBlock(GpuBufferConnector<T> *block, bool firstLevel)
{
    if(block->getSize()==0){
        cout << "why does this happen? this really should never ever happen" << endl;
        return;
        assert(0);
    }
    if(firstLevel){
        mutex.lock();
    }
    uint32_t size = block->size;
    uint32_t first = block->index;
    uint32_t last = first+size-1;
    //check if there is a starting or endpoint overlapping with this
    //in case stitch it together
    std::map<uint32_t,uint32_t>::iterator itStart =
            starts.find(last+1);
    std::map<uint32_t,uint32_t>::iterator itEnd =
            ends.find(first-1);


    if(itStart != starts.end()){
        //there is a free block directly after the one we want to insert:
        //increase the size of our current block
        uint32_t otherSize = itStart->second;
        uint32_t otherStart = last+1;
        block->size += otherSize;

        // now erase everything that leads up to this
        std::map<size_t,std::set<uint32_t>>::iterator it =
            freeSlots.find(otherSize);
        if(it == freeSlots.end()){
            assert(0);//this should never happen
            // if the structure were consistent there
            //has to be something found in here
        }

        set<uint32_t> &listFreeOfSize =
                it->second;

        if(listFreeOfSize.find(otherStart) == listFreeOfSize.end()){
            assert(0);
        }
        listFreeOfSize.erase(listFreeOfSize.find(otherStart));
        if(listFreeOfSize.empty()){
            freeSlots.erase(it);
        }

        starts.erase(itStart);
        ends.erase(ends.find(otherStart + otherSize - 1));

        //recursively call this function to reintroduce the block
        rejoinBlock(block, false);

        return;
    }
    if(itEnd != ends.end()){
        //there is a free block directly before the one we want to insert:
        //increase the size of our current block+
        //adapt the index
        uint32_t otherSize = itEnd->second;
        uint32_t otherStart = first - otherSize;
        block->index -= otherSize;
        block->size += otherSize;

        // now erase everything that leads up to this
        std::map<size_t,std::set<uint32_t>>::iterator it =
            freeSlots.find(otherSize);
        if(it == freeSlots.end()){
            assert(0);//this should never happen
            // if the structure were consistent there
            //has to be something found in here
        }

        set<uint32_t> &listFreeOfSize =
                it->second;
        if(listFreeOfSize.find(otherStart) == listFreeOfSize.end()){
            assert(0);//this should never happen either
        }
        listFreeOfSize.erase(listFreeOfSize.find(otherStart));
        if(listFreeOfSize.empty()){
            freeSlots.erase(it);
        }

        starts.erase(starts.find(otherStart));
        ends.erase(itEnd);

        //recursively call this function to reintroduce the block
        rejoinBlock(block, false);

        return;
    }

    //since we found everything we
    starts[first] = size;
    ends[last] = size;
    freeSlots[size].insert(first);



    mutex.unlock();
}

template<typename T>
void GpuBuffer<T>::upload(T *data, uint32_t index, uint32_t count)
{
    cudaError_t error_test;
    error_test = cudaMemcpy( &(cudaResourcePtr[index]),
                data,
                sizeof(T)*count,
                cudaMemcpyHostToDevice);
    gpuErrchk( error_test );

}

template<typename T>
void GpuBuffer<T>::download(T *data, uint32_t index, uint32_t count){

    cudaError_t error_test;
     error_test = cudaMemcpy( data,
                 &(cudaResourcePtr[index]),
                 sizeof(T)*count,
                 cudaMemcpyDeviceToHost);
     cudaDeviceSynchronize();
     gpuErrchk( error_test );
}

template<typename T>
uint32_t GpuBuffer<T>::getUsedElements() {
    mutex.lock();
    int count = 0;
    for (auto size : starts) {
        count += size.second;
    }


    mutex.unlock();
    return debugAbsoluteSize - count;
}

template<typename T>
uint32_t GpuBuffer<T>::getFreeElements(){

    mutex.lock();
    int count = 0;
    for (auto size : starts) {
        count += size.second;
    }


    mutex.unlock();
    return count;
}


template<typename T>
GpuBufferConnector<T>::GpuBufferConnector(GpuBuffer<T> *buffer,
                                          uint32_t index,
                                          uint32_t size):
    buffer(buffer),
    index(index),
    size(size){

}

template<typename T>
GpuBufferConnector<T>::~GpuBufferConnector()
{
    buffer->rejoinBlock(this);
}

template<typename T>
void GpuBufferConnector<T>::upload(T *data, uint32_t offset, uint32_t count)
{
    if(count==0){
        return;
    }
    buffer->upload(data,index + offset,count);
}

template<typename T>
void GpuBufferConnector<T>::download(T *data, uint32_t offset, uint32_t count)
{
    if(count==0){
        return;
    }
    buffer->download(data,index + offset,count);
}

/*
void testBuffer()
{

    GpuBuffer<uint8_t> buffer(100,true);

    std::shared_ptr<GpuBufferConnector<uint8_t>> blockA = buffer.getBlock(10);
    std::shared_ptr<GpuBufferConnector<uint8_t>> blockC = buffer.getBlock(10);
    blockA.reset();

    std::shared_ptr<GpuBufferConnector<uint8_t>> blockD = buffer.getBlock(9);

    std::shared_ptr<GpuBufferConnector<uint8_t>> blockE = buffer.getBlock(1);


    blockC.reset();


    std::shared_ptr<GpuBufferConnector<uint8_t>> blockB = buffer.getBlock(100);


    blockB.reset();
    blockD.reset();
    blockE.reset();

    cout << "how is the state of this buffer? " << endl;

}
*/


template class GpuBufferConnector<GpuVertex>;
template class GpuBufferConnector<Eigen::Vector2f>;
template class GpuBufferConnector<GpuTriangle>;
template class GpuBufferConnector<GpuPatchInfo>;
template class GpuBufferConnector<GLint>;


template class GpuBuffer<GpuVertex>;
template class GpuBuffer<Eigen::Vector2f>;
template class GpuBuffer<GpuTriangle>;
template class GpuBuffer<GpuPatchInfo>;
template class GpuBuffer<GLint>;

