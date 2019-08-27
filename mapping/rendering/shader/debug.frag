R"(
#version 450 core




layout(location = 0) out vec4 color;


void main(){
    color = vec4(0,1,1,1);

    //gl_Position = pos*10;
    //gl_Position.zw = vec2(0,1);
    //gl_Position = vec4(0,0,0,1);
}

)"
