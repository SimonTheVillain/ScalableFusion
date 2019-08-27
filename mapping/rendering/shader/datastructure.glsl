R"(

#version 450 core
#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_NV_gpu_shader5 :enable

//this is a quasiheader that defines the datastructures that are used by the opengl
//headers


//TODO: change that to something reasonable: 3 maybe?
const int GPU_MAX_TEX_PER_PATCH = 10;


struct GpuVertex{
 vec4 p;
 vec3 n;
 int16_t texIndex; //the texture index within the texture associated with its patch
 int16_t valid;
};

struct GpuTriangle{
    int16_t patchInfoInds[3];
    //int16_t placeholder1;
    int16_t posIndices[3];
    //int16_t placeholder2;
    int16_t texIndices[3];
    //int16_t placeholder3;


    //how to not render invalid vertices
    //https://stackoverflow.com/questions/5116289/is-it-possible-to-drop-a-triangle-from-being-rendered-using-vertex-shaders
    int16_t invalid;//invalid pixel which are valid
    int16_t alltogether;//pixel alltogether

};



//POSSIBILITY: Adding cuda surfaces and textures could increase utility of this
//structure
struct GpuTextureInfo{
    uint64_t glRefTexPtrDEBUG;
    vec2 refTexPosDEBUG;
    uint64_t glTexPointer;
    uint32_t texCoordStartInd;//could also be 16 bit int
    uint32_t placeholder;//could also be 16 bit int


    /**
     * Since the texture itself is shared with multiple other patches
     * we also need to store where the texture is on this texture atlas
     */
    //don't know yet what will be needed afterall
    vec2 pos;
//vec2 placeholder2;
    vec2 size;
//vec2 placeholder3;
    vec2 _size;
//vec2 placeholder4;
    //vec2 placeholder2;

};


//https://www.ibm.com/developerworks/aix/tutorials/au-memorymanager/
/**
 * Not a big structure that does not contain much for the triangles themselves,
 * only offsets and other stuff that might be useful not to do on the cpu.
 * But maybe it is not useful to calculate these offsets for each of the render passes.
 * maybe we first create an easily renderable structure via a cuda shader. (maybe by
 * storing vertex indices in a ringbuffer)
 */
struct GpuPatchInfo{
    int32_t patchId;
    int32_t triangleStartInd;
    int32_t vertexSourceStartInd;
    int32_t vertexDestinationStartInd;
    GpuTextureInfo stdTexture;
    GpuTextureInfo textureInfos[GPU_MAX_TEX_PER_PATCH];
    GpuTextureInfo semanticTextures[3];
    GpuTextureInfo segmentationTexture;//s[3];
    int32_t texLayers;
    int32_t semanticTexCount;
    int32_t segmentationTexCount;

    //TODO!!

    int32_t debug1;
    ///maybe we should add some flags for cases that one of the cuda kernels updated some of the
    /// geometry. This could easily be detected and give the chance to download updated geometry to the
    /// cpu.
};



)"
