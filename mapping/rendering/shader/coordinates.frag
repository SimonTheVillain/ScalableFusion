R"(
#version 450 core


#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 color;




flat in int triangleIndex;//thats an easy one
in vec3 barycentricWeights;//this is not (barycentric interpolation?)

void main(void)
{
    color=vec4(1,0,0,1);

    color.x=intBitsToFloat(triangleIndex);
    color.yzw=barycentricWeights;
    //color=vec4(1,0,0,1);
}
)"
