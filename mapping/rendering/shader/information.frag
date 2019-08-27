R"(
#version 450 core


#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 position;
layout(location = 1) out vec4 color;


//this guy seems to have a good approach to uniforms:
//https://www.opengl.org/discussion_boards/showthread.php/177593-Nvidia-bindless-textures-first-experiences-and-bugs

//this location is asking for trouble!!!!!!
//these should be in buffers not uniforms or even attributes??
//layout(location = 13) uniform uint64_t bindlessTexture; //doesn't this influence the transformation??
layout(location = 4) uniform mat4 _projMatrix;
//layout(location = 8) uniform mat4 _projMatrix;
//use the inverted projection matrix to restore the unprojected point coordinates.
//uniform sampler2D myTextureSampler;


in vec2 texPosOut;
in vec4 interpPosition;//TODO!!!! interpolation like this is not the right for geometry
in vec4 interpProj;//debug is this the same as gl_FragCoord
flat in uint64_t bindlessTexture;
flat in int isStitch;
in float _z;
void main(void)
{

    position=_projMatrix*interpProj;
    //bindless texture code:
    sampler2D s = sampler2D(bindlessTexture);
    color = texture(s,texPosOut); //scale the texPos to see a more dramatic effect
    //TODO: The red dots in the debug window seems to show that the geometric texture
    //gets sampled where the according lookup table was not written. (thus it doesn't get updated
    // where it should be)

    color.x = _z;
    color.x=color.x;
}
)"
