R"(


//The data is delivered in buffers
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


//some funny varyings altough some are flat
out vec2 texPosOut;
out vec4 interpPosition;
out vec4 interpProj;
flat out uint64_t bindlessTexture;


out vec2 labelPosOut;
flat out uint64_t bindlessLabelTexture;

out float _z;


out vec4 normalOut;


void main(void)
{
    //-------------new rendering----------------
    int id=gl_VertexID;
    int pointId=id%3;
    int triangleId=id/3;
    const GpuTriangle triangle=triangles[triangleId];
    GpuPatchInfo mainPatchInfo = patches[triangle.patchInfoInds[0]];

    GpuPatchInfo patchInfo = patches[triangle.patchInfoInds[pointId]];
    int vertexId = triangle.posIndices[pointId] +
                    patchInfo.vertexSourceStartInd;//*uni.nrVerticesPerSlot;
    vec4 point=vertices[vertexId].p;



    GpuTextureInfo texInfo = mainPatchInfo.textureInfos[0];
    bindlessTexture = texInfo.glTexPointer;


    uint32_t texPosInd = triangle.texIndices[pointId] + texInfo.texCoordStartInd;

    texPosOut = texCoords[texPosInd];
    //adapt the coordinate to the atlas
    texPosOut = vec2(texPosOut.x*texInfo.size.x,texPosOut.y*texInfo.size.y);
    texPosOut = texPosOut + texInfo.pos;


    interpPosition=viewMatrix*point;  //new rendering
    interpProj=projMatrix*interpPosition;
    _z=1.0/interpPosition.z;
    gl_Position = projMatrix*interpPosition;//projMatrix*



    //setup the readout for the label texture;
    texInfo = mainPatchInfo.segmentationTexture;
    bindlessLabelTexture = texInfo.glTexPointer;


    texPosInd = triangle.texIndices[pointId] + texInfo.texCoordStartInd;
    labelPosOut = texCoords[texPosInd];
    //adapt the coordinate to the atlas
    labelPosOut = vec2(texPosOut.x*texInfo.size.x,texPosOut.y*texInfo.size.y);
    labelPosOut = texPosOut + texInfo.pos;




    //calculate normal
    vec3 points[3];
    for(int i=0;i<3;i++){
        GpuPatchInfo patchInfo = patches[triangle.patchInfoInds[i]];
        int vertexId = triangle.posIndices[i] +
                patchInfo.vertexSourceStartInd;
        vec4 p = viewMatrix * vertices[vertexId].p;
        points[i]=p.xyz;
    }

    normalOut.xyz = normalize(cross(   points[1]-points[0],
                                    points[2]-points[0]));
    normalOut.w=1;


}
)"
