R"(

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


layout(location = 0) uniform int maxNrTexPoints; //this seems to be the only thing that is actually needed
layout(location = 1) uniform int infoSlot; //either from here or we again change the whole structure



flat out int vertexIndex;

void main(void)
{
    int id=gl_VertexID;
    vertexIndex=id;
    GpuVertex vert = vertices[id];
//This shader might not have been a good idea since we would have to
//rebind a shader each time we switch from triangles to other stuff. + it is not done yet


}


)"
