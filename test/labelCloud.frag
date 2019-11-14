R"(
#version 450 core
layout(location = 0) out vec4 color;


in vec4 colorIn;



void main(){
  color = colorIn;
  //color = vec4(1,0,0,1);
}

)"
