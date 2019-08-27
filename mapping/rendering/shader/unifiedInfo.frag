R"(
#version 450 core


#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 depth;
layout(location = 1) out vec4 color;
layout(location = 2) out vec4 normal;
layout(location = 3) out vec4 label;


//this guy seems to have a good approach to uniforms:
//https://www.opengl.org/discussion_boards/showthread.php/177593-Nvidia-bindless-textures-first-experiences-and-bugs

//this location is asking for trouble!!!!!!
//these should be in buffers not uniforms or even attributes??
//layout(location = 13) uniform uint64_t bindlessTexture; //doesn't this influence the transformation??
layout(location = 4) uniform mat4 _projMatrix;
//layout(location = 8) uniform mat4 _projMatrix;
//use the inverted projection matrix to restore the unprojected point coordinates.
//uniform sampler2D myTextureSampler;


in vec4 interpPosition;//TODO!!!! interpolation like this is not the right for geometry
in vec4 interpProj;//debug is this the same as gl_FragCoord?




flat in uint64_t bindlessTexture;
in vec2 texPosOut;


in vec2 labelPosOut;
flat in uint64_t bindlessLabelTexture;


in float _z;


in vec4 normalOut;



void main(void)
{

    //DEPTH //TODO: this is not correct for big triangles
    depth.x = 1.0f/_z;

    //COLOR:
    //position=_projMatrix*interpProj;
    //bindless texture code:
    sampler2D s = sampler2D(bindlessTexture);
    color = texture(s,texPosOut); //scale the texPos to see a more dramatic effect
    float r = color.z;// swap colors!!!!
    color.z = color.x;
    color.x = r;


    //NORMAL
    normal = normalOut;

    //LABEL
    //TODO: proper readout of labels
    int test =  700000;
    label = vec4(intBitsToFloat(test),1,0,1);


    if(bindlessLabelTexture == 0){

        label = vec4(intBitsToFloat(-1),1,1,1);//white for all the fails
    }else{
        sampler2D sampler = sampler2D(bindlessLabelTexture);
        vec4 surfaceLabels = texture(sampler,labelPosOut);
        label = vec4(surfaceLabels.x,1,0,1);
    }
    //rendering to 32 bit integers
    //https://www.gamedev.net/forums/topic/571627-how-to-render-32-bit-integer-into-textures/
}
)"
