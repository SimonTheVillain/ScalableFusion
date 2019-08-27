R"(
#version 450 core

layout (location = 0) in vec4 pos;


layout(location = 0)uniform mat4 projTrans;

layout(std430, binding = 0) buffer colorKeys
{
    vec4 colorKey[];
};

struct Vertex{
    vec4 pos;
    int index[20];
    float confidence[20];
};
layout(std430,binding = 1) buffer points{
    Vertex vertices[]
};

out vec4 colorIn;


void main(){
    gl_Position = projTrans*pos;
}

)"
