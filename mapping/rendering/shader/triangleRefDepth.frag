R"(
#version 450 core


#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 texRef;//triangle reference
layout(location = 1) out vec4 geometry;


//this guy seems to have a good approach to uniforms:
//https://www.opengl.org/discussion_boards/showthread.php/177593-Nvidia-bindless-textures-first-experiences-and-bugs


flat in int isStitch;//maybe later
flat in int patchId;
flat in int triangleIndex;


in vec4 pointWorld;



void main(void)
{


    texRef.z=2;
    texRef.w=3.0;
    texRef.x= intBitsToFloat(patchId);
    texRef.y= intBitsToFloat(triangleIndex);
    geometry = pointWorld;



}
)"
