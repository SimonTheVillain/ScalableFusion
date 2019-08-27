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



layout(location = 4) uniform int renderWireframe;
layout(location = 5) uniform int colorMode;
layout(location = 6) uniform int lightingMode;

//some funny varyings altough some are flat
out vec2 texPosOut;
out vec4 interpPosition;
out vec4 interpProj;
out vec4 normal;
flat out uint64_t bindlessTexture;
flat out int isStitch;
flat out int patchId;
out float _z;

flat out int32_t debug1;

flat out int32_t debug2;


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
    patchId = mainPatchInfo.patchId;
    debug1 = mainPatchInfo.debug1;
    //debug
    //patchId = int(mainPatchInfo.textureInfos[0].glTexPointer); //debug ( stdTexture

    GpuPatchInfo patchInfo = patches[triangle.patchInfoInds[pointId]];
    int vertexId = triangle.posIndices[pointId] +
                    patchInfo.vertexSourceStartInd;
    vec4 point = vertices[vertexId].p;
    normal.xyz = vertices[vertexId].n;


    GpuTextureInfo texInfo = mainPatchInfo.textureInfos[0];
    if(colorMode==2){
        texInfo = mainPatchInfo.stdTexture;
    }
    //texCoordsSlot = texInfo.texCoordStartInd;//take the tex coords of the first textures
    //texCoordSlotOut = int(texInfo.glTexPointer);//debug seems to be OK tough


    if(mainPatchInfo.texLayers==0 && colorMode==0){
        bindlessTexture = 0;
        //bindlessTexture = texInfo.glTexPointer;//debug
    }else{
        bindlessTexture = texInfo.glTexPointer;//take the texutre from first slot
    }


    isStitch=1;//unfortunately as it is right now we can't tell if a triangle is stitching
    //point=points[id];//debug (check if the ssbo works)
    //------------------------------------------

    uint32_t texPosInd = uint32_t(triangle.texIndices[pointId]) +
            texInfo.texCoordStartInd;
    texPosOut = texCoords[texPosInd];
    //adapt the coordinate to the tex atlas
    texPosOut = vec2(texPosOut.x*texInfo.size.x,texPosOut.y*texInfo.size.y);
    texPosOut = texPosOut+texInfo.pos;
    if(colorMode == 4){
        //in this case we put out the (COME ON WHY CAN'T YOU FINISH COMMENTS)
        texInfo = mainPatchInfo.stdTexture;
        bindlessTexture = texInfo.glRefTexPtrDEBUG;
        uint32_t texPosInd = uint32_t(triangle.texIndices[pointId]) +
                texInfo.texCoordStartInd;
        texPosOut = texCoords[texPosInd];
        //adapt the coordinate to the tex atlas
        texPosOut = vec2(texPosOut.x*texInfo.size.x,texPosOut.y*texInfo.size.y);
        texPosOut = texPosOut+texInfo.refTexPosDEBUG;
    }
    if(colorMode == 5){
        //mode 5 is the output of the first layer of label textures
        texInfo = mainPatchInfo.segmentationTexture;
        bindlessTexture = texInfo.glTexPointer;
        uint32_t texPosInd = uint32_t(triangle.texIndices[pointId]) +
            texInfo.texCoordStartInd;
        texPosOut = texCoords[texPosInd];
        //adapt the coordinate to the tex atlas
        texPosOut = vec2(texPosOut.x*texInfo.size.x,texPosOut.y*texInfo.size.y);
        //texPosOut = texPosOut + texInfo.pos;
        texPosOut = texPosOut + texInfo.refTexPosDEBUG;//texInfo.pos;

        bindlessTexture = texInfo.glRefTexPtrDEBUG;
    }

    interpPosition=viewMatrix*point;  //the position of the vertex in space (gets interpolated for the fragments)
    interpProj=projMatrix*interpPosition;
    _z=1.0/interpPosition.z;
    gl_Position = projMatrix*interpPosition;

    debug2=0;

    //for rendering the wireframe overlay: get points in screen coordinates
    //OpenGL 4 Shading Language cookbook page 231!!!!
    vec2 sc[3];//screen coords
    vec3 points[3];
    for(int i=0;i<3;i++){
        GpuPatchInfo patchInfo = patches[triangle.patchInfoInds[i]];
        int vertexId = triangle.posIndices[i] +
                patchInfo.vertexSourceStartInd;
        vec4 p = vertices[vertexId].p;
        points[i]=p.xyz;
        p = projMatrix*viewMatrix*p;
        sc[i] = vec2(p.xy/p.w);
        sc[i].x = sc[i].x*res.x;
        sc[i].y = sc[i].y*res.y;


        //checck if the vertex to this triangle is valid
        if(int(vertices[vertexId].valid)==0){
            //just for debug, draw triangles in certain color
            //debug2=1;

            // discard the whole triangle
            //gl_Position.w=0;
            //return;
        }
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

    //WHAAAAAT?
    //setting two of them to zero
    for(int i=0;i<3;i++){
        //if we are on point 0 the distance to line a would be non zero(=ha)
        //but the distances to the lines b and c would be zero.
        if(i!=pointId){
            distances[i]=0;
        }
    }

    if(lightingMode==1 || true){ //always create the normal
        //for flat shading we need the normal of the triangle
        normal.xyz = normalize(cross(   points[1]-points[0],
                                        points[2]-points[0]));
        normal.w=1;
    }
}
)"
