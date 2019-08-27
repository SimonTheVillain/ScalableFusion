#ifndef FILE_COALESCED_MEMORY_TRANSFER_H
#define FILE_COALESCED_MEMORY_TRANSFER_H
#include <vector>
#include "gpuMeshStructure.h"
//it might be beneficial to combine downloads and then do the copying on the cpu side.


//especially when downloading vertex data.
void downloadVertices(std::vector<GpuVertex*> gpuVertices,GpuVertex* data);


class CoalescedGpuTransfer{
public:
    struct Task{
        int start;
        int count;
        void* target;
    };
    template<typename T>
    static void upload(std::vector<T> source,std::vector<Task> tasks);


    //this is especially useful for the header information
    template<typename T>
    static void upload(std::vector<T> source,std::vector<T*> gpuDst);


    struct TaskD2D{
        size_t sourceIndex;
        size_t destinationIndex;
        size_t count;
    };
    //needed, e.g. when copying texture coordinates
    template<typename T>
    static void device2DeviceSameBuf(T* buffer, std::vector<TaskD2D> tasks);


    template<typename T>
    struct SetTaskTemp{//templated
        T* dst;
        T value;
    };
    template<typename T>
    static void upload(std::vector<SetTaskTemp<T>> tasks);


    template<typename T>
    struct CpyTaskTemp{//templated
        T* src;
        T* dst;
    };
    template<typename T>
    static void copy(std::vector<CpyTaskTemp<T>> tasks);



    struct DirectTask{
        void* src;
        void* dst;
        int byteCount;
    };
    static void download(std::vector<DirectTask> tasks);

};

#endif
