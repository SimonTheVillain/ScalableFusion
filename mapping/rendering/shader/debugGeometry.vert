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

layout(location = 3) uniform int overwriteStartInd;


vec2 res=vec2(1280,800);//this should be a uniform set outside of the shader
out float distances[3];

void main(void)
{
    //-------------new rendering----------------
    int id=gl_VertexID;
    int pointId=id%3;
    int triangleId=id/3;
    const GpuTriangle triangle=triangles[triangleId];

    //debug
    //gl_Position = projMatrix * viewMatrix* vertices[triangle.patchInfoInds[pointId]].p;
    //return;


    GpuPatchInfo mainPatchInfo = patches[triangle.patchInfoInds[0]];
    int patchId = mainPatchInfo.patchId;
    //debug
    //patchId = int(mainPatchInfo.textureInfos[0].glTexPointer); //debug ( stdTexture

    GpuPatchInfo patchInfo = patches[triangle.patchInfoInds[pointId]];
    int startInd = patchInfo.vertexSourceStartInd = patchInfo.vertexSourceStartInd;
    if(overwriteStartInd>=0){
        startInd = overwriteStartInd;
    }

    int vertexId = triangle.posIndices[pointId] +
                    startInd;
    vec4 point = vertices[vertexId].p;


    gl_Position = projMatrix*viewMatrix*point;



vec2 sc[3];//screen coords
    vec3 points[3];
    for(int i=0;i<3;i++){
        GpuPatchInfo patchInfo = patches[triangle.patchInfoInds[i]];
        startInd = patchInfo.vertexSourceStartInd = patchInfo.vertexSourceStartInd;
        if(overwriteStartInd>=0){
            startInd = overwriteStartInd;
        }
        int vertexId = triangle.posIndices[i] +
                startInd;
        vec4 p = vertices[vertexId].p;
        points[i]=p.xyz;
        p = projMatrix*viewMatrix*p;
        sc[i] = vec2(p.xy/p.w);
        sc[i].x = sc[i].x*res.x;
        sc[i].y = sc[i].y*res.y;

        if(int(vertices[vertexId].valid)==0){
            //just for debug, draw triangles in certain color
            //sort of discard the whole triangle
            gl_Position.w=0;
            return;
        }
    }


    float a=length(sc[1]-sc[2]);
    float b=length(sc[2]-sc[0]);
    float c=length(sc[1]-sc[0]);
    float alpha = acos((b*b+c*c - a*a) / (2.0*b*c));
    float beta = acos((a*a+c*c - b*b) / (2.0*a*c));
    distances[0] = abs(c*sin(beta));//ha
    distances[1] = abs(c*sin(alpha));//hb
    distances[2] = abs(b*sin(alpha));//hc


    for(int i=0;i<3;i++){
        //if we are on point 0 the distance to line a would be non zero(=ha)
        //but the distances to the lines b and c would be zero.
        if(i!=pointId){
            distances[i]=0;
        }
    }

}
)"
