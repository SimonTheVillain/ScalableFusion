R"(
#version 450 core

#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 color;

flat in int vertex_index;//thats an easy one

void main(void) {
    //lets store the vertex index, the 0 in the barycentric
    //weight section tells that this is a vertex index and not a triangle index
    color = vec4(intBitsToFloat(vertex_index), 0, 0, 0);
}
)"
