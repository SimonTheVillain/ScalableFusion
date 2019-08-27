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
//struct Patch{
//}patches[];
GpuPatchInfo patches[];
};

layout(location = 0) uniform mat4 viewMatrix;//one matrix is taking up 4 locations
layout(location = 1) uniform mat4 projMatrix;
//layout(location = 8) uniform Uniforms uniforms;
//layout(location = 2) uniform Uniforms uni;

//some funny varyings altough some are flat
out vec2 texPosOut;
out vec4 interpPosition;
out vec4 interpProj;
flat out uint64_t bindlessTexture;
flat out int isStitch;
flat out int patchId;
flat out int texCoordSlotOut;//debug
out float _z;


vec2 res=vec2(1280,800);
out float distances[3];
void main(void)
{
    //-------------new rendering----------------
    int id=gl_VertexID;
    int pointId=id%3;
    int triangleId=id/3;
    const GpuTriangle triangle=triangles[triangleId];
    GpuPatchInfo mainPatchInfo = patches[triangle.patchInfoInds[0]];
    patchId = mainPatchInfo.patchId;

    GpuPatchInfo patchInfo = patches[triangle.patchInfoInds[pointId]];
    int vertexId = triangle.posIndices[pointId] +
                    patchInfo.vertexSourceStartInd;//*uni.nrVerticesPerSlot;
    vec4 point=vertices[vertexId].p;
    //point = vec4(triangle.posIndices[triangleId]*0.0001,0,0,1);
    GpuTextureInfo texInfo = mainPatchInfo.stdTexture;
    //uint32_t texCoordsSlot = texInfo.texCoordSlot;
    texCoordSlotOut = int(texInfo.texCoordStartInd);//debug seems to be OK tough
    bindlessTexture = texInfo.glTexPointer;

    isStitch=1;//unfortunately as it is right now we can't tell if a triangle is stitching
    //point=points[id];//debug (check if the ssbo works)
    //------------------------------------------

    uint32_t texPosInd = triangle.texIndices[pointId] + texInfo.texCoordStartInd;
    //texCoordSlotOut = int(uni.nrTexPointsPerSlot);//debug seems to be OK tough
    //texPosInd = triangle.posIndices[pointId];//debug: this only works as long as there is only one texture per patch.

    texPosOut = texCoords[texPosInd];
    texPosOut = vec2(texPosOut.x*texInfo.size.x,texPosOut.y*texInfo.size.y);
    texPosOut = texPosOut+texInfo.pos;


    interpPosition=viewMatrix*point;  //new rendering
    interpProj=projMatrix*interpPosition;
    _z=1.0/interpPosition.z;
    gl_Position = projMatrix*interpPosition;//projMatrix*



/*
    //for rendering the wireframe overlay: get points in screen coordinates
    //OpenGL 4 Shading Language cookbook page 231!!!!
    vec2 sc[3];//screen coords
    for(int i=0;i<3;i++){
        vec4 p=points[triangle.posIndices[i]].p;
        p=projMatrix*viewMatrix*p;
        sc[i]=vec2(p.xy/p.w);
        sc[i].x=sc[i].x*res.x;
        sc[i].y=sc[i].y*res.y;
    }

    //calculate heights
    float a=length(sc[1]-sc[2]);
    float b=length(sc[2]-sc[0]);
    float c=length(sc[1]-sc[0]);
    float alpha = acos((b*b+c*c - a*a) / (2.0*b*c));
    float beta = acos((a*a+c*c - b*b) / (2.0*a*c));
    distances[0] = abs(c*sin(beta));//ha
    distances[1] = abs(c*sin(alpha));//hb
    distances[2] = abs(b*sin(alpha));//hc
    //setting two of them to zero
    for(int i=0;i<3;i++){
        //if we are on point 0 the distance to line a would be non zero(=ha)
        //but the distances to the lines b and c would be zero.
        if(i!=pointId){
            distances[i]=0;
        }
    }
*/
    //putting all points to the center
    //gl_Position = vec4(0,0,0,1);


    //coloring by patch id!!!

/*
    gl_Position.x=gl_Position.x/gl_Position.w;
    gl_Position.y=gl_Position.y/gl_Position.w;
    gl_Position.z=0;//gl_Position.z/gl_Position.w;

    gl_Position.w=1.0;
    if(pos.z>0){
        //gl_Position.z=10;
    }
*/

//For vertex positions just use the inverse of the view projection matrix etc
    //gl_Position = pos;
    /*gl_Position.z=0;
    gl_Position.x*=0.1;
    gl_Position.y*=0.1;
    gl_Position.w=1;
*/
}
)"
