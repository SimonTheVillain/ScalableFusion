R"(
#version 450 core




layout(location = 0) out vec4 texRef;
layout(location = 1) out vec4 geometry;
in vec4 colorIn;
in vec4 normalIn;
in vec4 geometryIn;


void main(){
    texRef = vec4(0,0,0,10.0f);
    geometry = geometryIn;
    //gl_Position = pos*10;
    //gl_Position.zw = vec2(0,1);
    //gl_Position = vec4(0,0,0,1);
}

)"
