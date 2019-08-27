R"(



//the data comes in via array buffers
layout(std430, binding=0) buffer VertexBuffer{
    GpuVertex vertices[];
};

layout(std430, binding=1) buffer TexCoordBuffer{
    vec2 texCoords[];
};

layout(std430, binding=2) buffer TriangleBuffer{
    GpuTriangle triangles[];
};


layout(std430, binding=3) buffer PatchBuffer{
//struct Patch{
//}patches[];
GpuPatchInfo patches[];
};

//layout(location = 0) uniform mat4 viewMatrix;//one matrix is taking up 4 locations
//layout(location = 4) uniform mat4 projMatrix;
//layout(location = 8) uniform Uniforms uniforms;
layout(location = 0) uniform int maxNrTexPoints; //this seems to be the only thing that is actually needed


//todo: The output for the whole shader is going to be a texture with triangle indices and
//a parametrized position on the textures
flat out int triangleIndex;//thats an easy one
out vec3 barycentricWeights;//this is not (barycentric interpolation?)

void main(void)
{
    //-------------new rendering----------------
    int id=gl_VertexID;
    int pointId=id%3;
    int triangleId=id/3;
    const GpuTriangle triangle=triangles[triangleId];
    int patchSlot = triangle.patchInfoInds[0];
    GpuPatchInfo patchInfo = patches[patchSlot];

    vec4 point=vertices[triangle.posIndices[pointId]].p;


    uint32_t texPosInd = triangle.texIndices[pointId] +
                            patchInfo.stdTexture.texCoordStartInd;

    //the new stuff:
    gl_Position = vec4(texCoords[texPosInd]*2.0-vec2(1.0,1.0),0,1);
    //gl_Position = vec4(0,0,0,1);//DEBUG
    triangleIndex = triangleId;

    //TODO: test if this really does barycentric interpolation in opengl
    barycentricWeights = vec3(0,0,0);
    barycentricWeights[pointId]=1;


}


)"
