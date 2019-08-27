R"(
#version 450 core

layout (location = 0) in vec4 pos;


layout(location = 0)uniform mat4 projTrans;



void main(){
    gl_Position = projTrans*pos;

    //gl_Position = pos*10;
    //gl_Position.zw = vec2(0,1);
    //gl_Position = vec4(0,0,0,1);
}

)"
