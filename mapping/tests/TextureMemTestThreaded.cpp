//
// Created by simon on 3/8/19.
//
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <memory>

#include "../../gfx/gpuTex.h"
#include "../../gfx/GarbageCollector.h"
#include "../../gfx/glUtils.h"
#include "../gpu/threadSafeFBO_VAO.h"

//mostly because we need to sleep for a longer time
#include <chrono>
#include <thread>
#include <thread>
using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;

using namespace std;


int main(int argc, const char * argv[])
{
    cout << "starting allocating tons of memory on a single thread" << endl;
    GarbageCollector garbageCollector;
    glfwInit();
    GLFWwindow* invisibleWindow;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_VISIBLE, 0);
    invisibleWindow = glfwCreateWindow(640, 480, "HOPE U NO VISIBLE",
                                       nullptr, nullptr);

    glfwMakeContextCurrent(invisibleWindow);
    glewExperimental = GL_TRUE;
    glewInit();
    glGetError();


    GLFWwindow* otherContext;
    otherContext = glfwCreateWindow(640, 480, "HOPE U NO VISIBLE",
                                       nullptr, invisibleWindow);


    vector<shared_ptr<gfx::GpuTex2D>> textures;
    int fullRes=1024;
    GLuint intType = GL_RGBA32F;
    GLuint format = GL_RGBA;
    GLuint type = GL_FLOAT;
    ThreadSafeFBOStorage fboStorage;
    vector<ThreadSafeFBO*> fbos;
    vector<uint64_t> handles;
    bool done=false;
    auto func = [&](){

        if (!glfwInit())
            exit(EXIT_FAILURE);
        glfwMakeContextCurrent(otherContext);
        glewExperimental = GL_TRUE;
        glewInit();
        glGetError();

        for(int i=0;i<400;i++){

            auto tex =
                    make_shared<gfx::GpuTex2D>(
                            &garbageCollector,
                            intType,format,type,
                            fullRes,fullRes,
                            true,
                            nullptr);
            glMakeTextureHandleResidentARB(tex->getGlHandle());
            handles.push_back(tex->getGlHandle());
            auto fbo = fboStorage.createGlObject();
            glBindFramebuffer(GL_FRAMEBUFFER,fbo->get());
            glBindTexture(GL_TEXTURE_2D,tex->getGlName());//not necessary
            glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,tex->getGlName(),0);
            GLenum drawBuffers[1]={GL_COLOR_ATTACHMENT0};
            glDrawBuffers(1,drawBuffers);

            GLenum glErr=glCheckFramebufferStatus( GL_FRAMEBUFFER);
            if(glErr != GL_FRAMEBUFFER_COMPLETE){
                cout << "framebuffer incomplete" << endl;
            }
            fbos.push_back(fbo);
            //TODO: also get a fbo
            //TODO: also make them resident!!!!!!!

            textures.push_back(tex);


            if(glIsTextureHandleResidentARB(tex->getGlHandle())){
                //glMakeTextureHandleNonResidentARB(tex->getGlHandle());
            }else{
                cout << "why should it not be resident?" << endl;
            }
        }
        fboStorage.forceGarbageCollect();

        gfx::GLUtils::checkForOpenGLError("is making texture resident a big mistake?");
        done = true;

    };
    thread t(func);
    while(!done){

    }
    for(int i=0;i<400;i++){
        auto tex = textures[i];
        auto fbo = fboStorage.createGlObject();
        glBindFramebuffer(GL_FRAMEBUFFER,fbo->get());
        glBindTexture(GL_TEXTURE_2D,tex->getGlName());//not necessary
        glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,tex->getGlName(),0);
        GLenum drawBuffers[1]={GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1,drawBuffers);
        GLenum glErr=glCheckFramebufferStatus( GL_FRAMEBUFFER);
        if(glErr != GL_FRAMEBUFFER_COMPLETE){
            cout << "framebuffer incomplete" << endl;
        }
        fbos.push_back(fbo);
        gfx::GLUtils::checkForOpenGLError("is making texture resident a big mistake?");

    }

    t.join();


    sleep_for(seconds(10));


    for(int i=0;i<400;i++){
        auto tex = textures[i];
        //seemingly these texture handles need to be made resident per thread!!!
        /*
        if(glIsTextureHandleResidentARB(tex->getGlHandle())){
            glMakeTextureHandleNonResidentARB(tex->getGlHandle());
        }else{
            cout << "why should it not be resident?" << endl;
        }
         */

    }
    for(size_t i=0;i<handles.size();i++){
        glMakeTextureHandleNonResidentARB(handles[i]);
        gfx::GLUtils::checkForOpenGLError("is making texture non resident a big mistake?");

    }
    textures.clear();
    //destroy
    cout << "textures should be cleared" << endl;

    sleep_for(seconds(10));
    for(auto fbo : fbos){
        delete fbo;
    }
    fbos.clear();
    fboStorage.garbageCollect();
    glFinish();
    cout << "fbos should be cleared" << endl;

    sleep_for(seconds(10));


    for(size_t i=0;i<handles.size();i++){
        //glMakeTextureHandleNonResidentARB(handles[i]);
        gfx::GLUtils::checkForOpenGLError("is making texture non resident a big mistake?");

    }
    cout << "making the textures non-resident" << endl;
    sleep_for(seconds(10));


    glfwDestroyWindow(invisibleWindow);
    glfwTerminate();
    cout << "OpenGLContext should be cleared" << endl;

    sleep_for(seconds(10));

    cout << "It is over now" << endl;



    return 0;

}