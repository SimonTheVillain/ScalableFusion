R"(
#version 450 core
#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_NV_gpu_shader5 :enable

//plan is to get rid of these two:
//layout(location = 0) in vec4 pos;
//layout(location = 1) in vec2 texPos;

const int GPU_MAX_TEX_PER_PATCH = 100;


struct GpuVertex{
 vec4 p;
 vec4 n;
};

//the new source of data!!!
layout(std430, binding=0) buffer VertexBuffer{
    GpuVertex vertices[];
};

layout(std430, binding=1) buffer TexCoordBuffer{
    vec2 texCoords[];
};
struct GpuTriangle{
    int32_t ind[3];
    int16_t placeholder1[3];
    int16_t placeholder2[3];
};
layout(std430, binding=2) buffer TriangleBuffer{
    GpuTriangle triangles[];
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
    GpuTextureInfo stdTexture;
    GpuTextureInfo textureInfos[GPU_MAX_TEX_PER_PATCH];
    ///maybe we should add some flags for cases that one of the cuda kernels updated some of the
    /// geometry. This could easily be detected and give the chance to download updated geometry to the
    /// cpu.
};



layout(std430, binding=3) buffer PatchBuffer{
//struct Patch{
//}patches[];
GpuPatchInfo patches[];
};

layout(location = 0) uniform mat4 viewMatrix;//one matrix is taking up 4 locations
layout(location = 4) uniform mat4 projMatrix;
//layout(location = 8) uniform Uniforms uniforms;
layout(location = 8) uniform int maxNrTexPoints;


void main(void)
{
    gl_Position = vertices[gl_VertexID].p;
    gl_Position = projMatrix*viewMatrix*gl_Position;//projMatrix*

    //now the triangle version of this:

    GpuTriangle triangle = triangles[gl_VertexID/3];


    gl_Position = vertices[triangle.ind[gl_VertexID%3]].p;
    gl_Position = projMatrix*viewMatrix*gl_Position;//projMatrix*


}
)"
