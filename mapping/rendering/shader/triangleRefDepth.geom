R"(
/******
A mere placeholder.
*/
#version 450 core
#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_NV_gpu_shader5 :enable

//this is a quasiheader that defines the datastructures that are used by the opengl
//headers

const int GPU_MAX_TEX_PER_PATCH = 100;


struct GpuVertex{
 vec4 p;
 vec3 n;
 int texIndex; //the texture index within the texture associated with its patch
};

struct GpuTriangle{
    int16_t patchInfoSlots[3];
    //int16_t placeholder1;
    int16_t posIndices[3];
    //int16_t placeholder2;
    int16_t texIndices[3];
    //int16_t placeholder3;
};
struct GpuTextureInfo{
    uint64_t glTexPointer;
    uint32_t texCoordSlot;
    uint32_t placeholder;

    vec2 pos;
    vec2 size;
    vec2 _size;

};
struct GpuPatchInfo{
    int32_t patchId;
    int32_t triangleSlot;
    int32_t vertexSourceSlot;
    int32_t vertexDestinationSlot;
    GpuTextureInfo stdTexture;
    GpuTextureInfo textureInfos[GPU_MAX_TEX_PER_PATCH];
    int32_t texLayers;
    int32_t placeholder;
    ///maybe we should add some flags for cases that one of the cuda kernels updated some of the
    /// geometry. This could easily be detected and give the chance to download updated geometry to the
    /// cpu.
};


struct Uniforms{//this will take up 2 locations
    int32_t nrTexPointsPerSlot;
    int32_t nrVerticesPerSlot;
};

)"
