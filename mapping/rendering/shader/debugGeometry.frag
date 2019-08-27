R"(
#version 450 core


#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

layout(location = 0) out vec4 colorOut;

layout(location = 2) uniform vec4 color;



in float distances[3];
float lineWidth=0.9;//0.5;//1.5;
vec4 lineColor=vec4(0,0,0,1);


void main(void)
{
    colorOut = color;//vec4(1.0,0.1,0.1,1);



    float minDistance=min(distances[0],min(distances[1],distances[2]));
    float mixVal=smoothstep(lineWidth-0.5,lineWidth+0.5,minDistance);
    colorOut=mix(lineColor,color,mixVal);

}
)"
