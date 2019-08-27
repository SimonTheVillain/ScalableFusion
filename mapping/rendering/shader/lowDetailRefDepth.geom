R"(
#version 450 core

layout (triangles) in;

layout (triangle_strip,max_vertices = 3) out;
//layout (points,max_vertices = 3) out;


in vec4 points[];
in vec4 colors[];
in vec4 normals[];
in int valids[];

out vec4 colorIn;
out vec4 normalIn;
out vec4 geometryIn;



void main(){
    if(valids[0]==0 && valids[1]==0 && valids[2]==0){
        //don't do anything if the vertices are not all valid
        return; //debug
    }
    for(int i=0;i<3;i++){

        colorIn = colors[i];
        normalIn= normals[i];
        geometryIn = points[i];
        gl_Position = gl_in[i].gl_Position;


        EmitVertex();
    }

    EndPrimitive();


    //gl_Position = pos*10;
    //gl_Position.zw = vec2(0,1);
    //gl_Position = vec4(0,0,0,1);
}

)"
