#ifndef FILE_GPU_MESH_STRUCTURE_H
#define FILE_GPU_MESH_STRUCTURE_H
#include <stdint.h>
#include <Eigen/Eigen>


#define GPU_MAX_TEX_PER_PATCH 10

//TODO: maybe p could be a Vector3f,
//n could be 3 bytes and
//the tex index in main patch..... if this would not be used it could half
//the space needed (forget it then!)
struct GpuVertex{
    Eigen::Vector4f p;
    Eigen::Vector3f n;

    //the texture index in the tex coordinates of the main patch
    int16_t texIndInMainPatch;//could also be 16 bit int

    //if any of a triangle vertex is invalid we should not render it
    int16_t valid=1;
};


//triangle with references to everything. since the first referenced patch(patch slot) is the one we get
//our textures from we do not need extra structures for stitch triangles.
struct GpuTriangle{

    //TODO: transition from absolute indices to relative indices and
    //references to the patch itself.
    int16_t patchInfoInds[3];//patch info indices for the triangle
    //int16_t placeholder1;

    int16_t indices[3];//these are absolute indices for the vertex buffer.

    //int16_t placeholder2;
    int16_t texIndices[3];//these texIndices are relative. the texture information itself is within the patchSlot
    //int16_t placeholder3;

    //todo:
    int16_t invalid=0;//invalid pixel which are valid
    int16_t alltogether=0;//pixel alltogether
    //if too many of the pixel are invalid
};


///TODO: maybe we add slots for CUDA surfaces and stuff like that.
struct GpuTextureInfo{
    uint64_t glRefTexPtrDEBUG;
    Eigen::Vector2f refTexPosDEBUG;
    uint64_t glTexPointer;
    uint32_t texCoordStartInd;//could also be 16 bit int
    uint32_t placeholder;//could also be 16 bit int


    //TOIMPLEMENT
    /**
     * Since the texture itself is shared with multiple other patches
     * we also need to store where the texture is on this
     */
    //don't know yet what will be needed afterall
    Eigen::Vector2f pos;
    //Eigen::Vector2f placeholder2;
    Eigen::Vector2f size;
    //Eigen::Vector2f placeholder3;
    Eigen::Vector2f _size;
    //Eigen::Vector2f placeholder4;

};//__attribute__ ((aligned(16)));//this is needed even though it doesn't work
//there is some padding/ alignment issues!

//https://www.ibm.com/developerworks/aix/tutorials/au-memorymanager/
/**
 * Not a big structure that does not contain much for the triangles themselves,
 * only offsets and other stuff that might be useful not to do on the cpu.
 * But maybe it is not useful to calculate these offsets for each of the render passes.
 * maybe we first create an easily renderable structure via a cuda kernel. (maybe by
 * storing vertex indices in a ringbuffer)
 */
struct  GpuPatchInfo{
    int32_t patchId;
    int32_t triangleStartInd;
    int32_t vertexSourceStartInd;
    int32_t vertexDestinationStartInd;
    GpuTextureInfo stdTexture;
    GpuTextureInfo textureInfos[GPU_MAX_TEX_PER_PATCH];
    static const int maxSemTexCnt = 3;//shouldn't take space
    GpuTextureInfo semanticTextures[maxSemTexCnt];


    //static const int maxSegTexCnt = 3;//shouldn't take space
    GpuTextureInfo segmentationTexture; //TODO add a bool to show this actually exists
    int32_t texLayers;
    int32_t semanticTexCount;
    int32_t segmentationTexValid;



    int32_t debug1=0;
};//__attribute__ ((aligned(16)));



struct GpuCoarseVertex{
    Eigen::Vector4f p;
    Eigen::Vector4f n;
    Eigen::Vector4f c;
};

#endif
