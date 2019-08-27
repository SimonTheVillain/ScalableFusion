R"(

//the new source of data!!!
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
    GpuPatchInfo patches[];
};

layout(location = 0) uniform mat4 viewMatrix;//one matrix is taking up 4 locations
layout(location = 1) uniform mat4 projMatrix;
//layout(location = 8) uniform Uniforms uniforms;

/*
struct Uniforms{//this will take up 2 locations
    int32_t nrTexPointsPerSlot;
    int32_t nrVerticesPerSlot;
};
*/

//some funny varyings altough some are flat
flat out int isStitch;
flat out int patchId;
flat out int triangleIndex;
out vec4 pointWorld;


void main(void)
{
    //-------------new rendering----------------
    int id=gl_VertexID;
    int pointId=id%3;
    int triangleId=id/3;
    const GpuTriangle triangle=triangles[triangleId];
    GpuPatchInfo mainPatchInfo = patches[triangle.patchInfoInds[0]];
    patchId = mainPatchInfo.patchId;
    triangleIndex = triangleId - mainPatchInfo.triangleStartInd;

    GpuPatchInfo patchInfo = patches[triangle.patchInfoInds[pointId]];
    int vertexId = triangle.posIndices[pointId] +
                    patchInfo.vertexSourceStartInd;
    vec4 point = vertices[vertexId].p;
    pointWorld=point;
    //point = vec4(0,0,0,1);//DEBUG: PLEASE REMOVE
    //point = vec4(triangle.posIndices[triangleId]*0.0001,0,0,1);//ANOTHER DEBUG
    isStitch=10;



    vec4 interpPosition=viewMatrix*point;  //the position of the vertex in space (gets interpolated for the fragments)
    gl_Position = projMatrix*interpPosition;//projMatrix*

    gl_Position.y*=-1;


}
)"
