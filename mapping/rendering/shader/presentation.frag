R"(
#version 450 core


#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 color;


//this guy seems to have a good approach to uniforms:
//https://www.opengl.org/discussion_boards/showthread.php/177593-Nvidia-bindless-textures-first-experiences-and-bugs


in vec2 texPosOut;
in vec4 interpPosition;//TODO!!!! interpolation like this is not the right for geometry
in vec4 interpProj;//debug is this the same as gl_FragCoord
in vec4 normal;
flat in uint64_t bindlessTexture;
flat in int isStitch;
flat in int patchId;
flat in int texCoordSlotOut;//thats a debug thingy
in float _z;

layout(location = 4) uniform int renderWireframe;
layout(location = 5) uniform int colorMode;
layout(location = 6) uniform int lightingMode;
layout(location = 7) uniform vec4 lineColor;


//this is needed for
in float distances[3];
flat in int32_t debug1;
float lineWidth=0.9;//0.5;//1.5;
//vec4 lineColor=vec4(0,0,0,1);

flat in int32_t debug2;



vec4 colorCode(int32_t id){
    uint8_t r=uint8_t((id/10*50)%255);
    uint8_t g=uint8_t((id%7)*30);
    uint8_t b=uint8_t((id%5)*50);
    return vec4(float(b)/255.0,float(g)/255.0,float(r)/255.0,1.0);
}



void main(void)
{
    if(debug2==1){
        color=vec4(0.7,0.7,0,1);
        return;
    }
    //debug: output only read
    //color = vec4(1,0,0,1);
    //return; //debug

    if(debug1==1){
        color=vec4(0,1,0,1);
        return;
    }

    if(bindlessTexture==0){
        //if there is no texture bound we put out a other color
        color=vec4(0.5,0.5,0,1);
        return;
    }else{
        sampler2D s = sampler2D(bindlessTexture);
        color = texture(s,texPosOut);
    }
    //bindless texture code:
/*

    //debug code for tex coordinates
    vec2 debugPixCoord = gl_FragCoord.xy;
    debugPixCoord = vec2(debugPixCoord.x/1280,debugPixCoord.y/800);
    //color = texture(s,debugPixCoord);
    //color = vec4(debugPixCoord,0,1);
    //classical
    //color = texture( myTextureSampler, texPosOut )
    float switcharoo=color.x;
    //color.x=color.z;
    //color.z=switcharoo;
    color.w=1;


    */
    //rendering the wireframe
    float minDistance=min(distances[0],min(distances[1],distances[2]));
    float mixVal=smoothstep(lineWidth-0.5,lineWidth+0.5,minDistance);

    //color=mix(lineColor,color,mixVal);

    //coloring based on patch id
    //uint8_t r=uint8_t((patchId/10*50)%255);
    //uint8_t g=uint8_t(((patchId%7))*30);
    //uint8_t b=uint8_t((patchId%5)*50);

    if(colorMode==1){
        color=colorCode(patchId);//vec4(float(b)/255.0,float(g)/255.0,float(r)/255.0,1.0);
    }else if(colorMode==2){
        //presenting the standard deviation textures....
        //multiplying with 10 is a good idea there
        color.xyz=color.xyz*10.0;
    }else if(colorMode==3){
        color= -normal*0.5 + vec4(0.5,0.5,0.5,0.5);
        color.w=1.0;
    }else if(colorMode==4){
        //color.xyz=color.xyz*10.0;

        if(floatBitsToInt(color.x)<0){
            color.xyz = vec3(1,0,0);
        }else{
            color.xyz=color.yzw;
        }

        color.w=1.0;
    }else if(colorMode == 5){
        if(true){
            if(floatBitsToInt(color.x)<0){
                color.xyz = vec3(1,0,0);
            }else{
                color.xyz=color.yzw;
            }
        }

        return;//debug: show the raw texture
        int segmentLabel = floatBitsToInt(color.x);
        //color=colorCode(segmentLabel);
        color = vec4(color.x,0,0,1);//another debug
        //color=vec4(1,0,0,1);//debug
        color = vec4(texPosOut,0,1);//why is this flickering? why!!!!!!
    }

    //certain debug outputs
    if(isStitch!=0){
        //color=vec4(0,0,0,1);
    }
    if(texCoordSlotOut==10){
        //color = vec4(1,0,0,1);
    }

    if(lightingMode==0){
        //leave it as it is
    }
    if(lightingMode==1){
        //flat shading
//debug
        color=normal*0.5 + vec4(0.5,0.5,0.5,0.5);
        color.w=1;

    }
    if(lightingMode==2){
        //phong shading
    }
    if(lightingMode==3){
        color=vec4(texPosOut,0,1);//debug
    }





    //color=mix(lineColor,color,mixVal);

    if(renderWireframe!=0){
        color=mix(lineColor,color,mixVal);
    }

    //color=vec4(texPosOut,0,1);//debug

}
)"
