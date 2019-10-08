#include "gpuTex.h"
#include "glUtils.h"
#include "GarbageCollector.h"
#include <iostream>
#include <stdio.h>
#include <assert.h>

#include <cuda.h>
#include <cuda_gl_interop.h>

#include <string.h>//mainly because of memset
//#include <opencv2/cudaimgproc.hpp>


#include <chrono>


using namespace std;
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) assert(false);
   }
}

//void copy(cudaTextureObject_t texture,cv::cuda::GpuMat &to);



int gfx::GpuTex2D::overallTexCount = 0;
int gfx::GpuTex2D::getTexCount()
{
    return overallTexCount;
}

std::mutex gfx::GpuTex2D::overallTexListMutex;
std::vector<gfx::GpuTex2D*> gfx::GpuTex2D::overallTexList;

std::vector<gfx::GpuTex2D*> gfx::GpuTex2D::getTexList() {
    return overallTexList;
}

gfx::GpuTex2D::GpuTex2D(GarbageCollector* garbageCollector,GLuint glInternalFormat, GLuint glFormat, GLuint glType, int width, int height,
        bool cudaNormalizedTexCoords, void *data,GLint filterType)
{
    this->garbageCollector = garbageCollector;
    if(width == 640 && height == 480){
       // cout << "we got one" << endl;
    }

    overallTexListMutex.lock();
    overallTexCount++;
    overallTexList.push_back(this);
    overallTexListMutex.unlock();
    auto start = chrono::high_resolution_clock::now();
    this->m_width=width;
    this->m_height=height;
    this->m_glInternalFormat=glInternalFormat;
    this->m_glFormat=glFormat;
    this->m_glType=glType;


    gfx::GLUtils::checkForOpenGLError("Error before generating glTexture");
    glGenTextures(1,&glName);

    gfx::GLUtils::checkForOpenGLError("Error while generating glTexture");
    glBindTexture(GL_TEXTURE_2D,glName);
    glTexImage2D(GL_TEXTURE_2D, 0, glInternalFormat, width, height, 0, glFormat,
                  glType, data);

    gfx::GLUtils::checkForOpenGLError("Error while creating glTexture");

    auto endGlTex =  chrono::high_resolution_clock::now();
    cudaChannelFormatKind type;
    int bitDepth = 0;
    switch(glType){
    case GL_UNSIGNED_BYTE:
        type = cudaChannelFormatKindUnsigned;
        bitDepth=8;
        byteCount=1;
        break;

    case GL_BYTE:
        type = cudaChannelFormatKindSigned;
        bitDepth=8;
        byteCount=1;
        break;

    case GL_UNSIGNED_SHORT:
        type = cudaChannelFormatKindUnsigned;
        bitDepth=16;
        byteCount=2;
        break;

    case GL_SHORT:
        type = cudaChannelFormatKindSigned;
        bitDepth=16;
        byteCount=2;
        break;

    case GL_UNSIGNED_INT:
        type = cudaChannelFormatKindUnsigned;
        bitDepth=32;
        byteCount=4;
        break;

    case GL_INT:
        type = cudaChannelFormatKindSigned;
        bitDepth=32;
        byteCount=4;
        break;

    case GL_FLOAT:
        type = cudaChannelFormatKindFloat;
        bitDepth=32;
        byteCount=4;
        //TODO: distinguish between float16 and float32
        if(glInternalFormat == GL_RGBA16F||
                glInternalFormat == GL_RGB16F||
                glInternalFormat == GL_RG16F||
                glInternalFormat == GL_R16F){
            bitDepth=16;
            byteCount=2;
        }
        break;
    default:

        break;
    }

    //i forgot, we don't need this
    cudaChannelFormatDesc channelDesc;
    switch(glFormat){ // i think i would be better of with if else statements
    case GL_RED:
        channelDesc=cudaCreateChannelDesc(bitDepth, 0, 0, 0,type);
        channelCount=1;
        break;
    case GL_RG:
        channelDesc=cudaCreateChannelDesc(bitDepth, bitDepth, 0, 0,type);
        channelCount=2;
        break;
    case GL_RGB:
        channelDesc=cudaCreateChannelDesc(bitDepth, bitDepth, bitDepth, 0,type);
        channelCount=3;
        break;
    case GL_RGBA:
        channelDesc=cudaCreateChannelDesc(bitDepth, bitDepth, bitDepth, bitDepth,type);
        channelCount=4;
        break;
        //integer input
    case GL_RED_INTEGER:
        channelDesc=cudaCreateChannelDesc(bitDepth, bitDepth, bitDepth, bitDepth,type);
        channelCount=1;
        break;
    case GL_RG_INTEGER:
        channelDesc=cudaCreateChannelDesc(bitDepth, bitDepth, bitDepth, bitDepth,type);
        channelCount=2;
        break;
    case GL_RGB_INTEGER:
        channelDesc=cudaCreateChannelDesc(bitDepth, bitDepth, bitDepth, bitDepth,type);
        channelCount=3;
        break;
    case GL_RGBA_INTEGER:
        channelDesc=cudaCreateChannelDesc(bitDepth, bitDepth, bitDepth, bitDepth,type);
        channelCount=4;
        break;

    default:
        std::cout << "Invalid channel count!"<< std::endl;
        assert(0);
        break;
    }


    //register resource
    cudaGraphicsGLRegisterImage( &cudaTextureResource,
                                                    glName,
                                                    GL_TEXTURE_2D,
                                                    cudaGraphicsRegisterFlagsNone);

    auto endCudaGLRegister = chrono::high_resolution_clock::now();
    gpuErrchk( cudaPeekAtLastError() );

    //map resource
    cudaGraphicsMapResources(1 ,
                             &cudaTextureResource,
                             0);
    gpuErrchk( cudaPeekAtLastError() );

    //get array from this mapped resource
    cudaGraphicsSubResourceGetMappedArray(&cudaArrayReference,
                                          cudaTextureResource,
                                          0,0);
    gpuErrchk( cudaPeekAtLastError() );

    //create this cuda texture:
    cudaResourceDesc resDesc;
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypeArray;
    resDesc.res.array.array = cudaArrayReference;//this is the only one that is allowed for binding it to a texture or surface

    cudaTextureDesc texDesc;
    memset(&texDesc, 0, sizeof(texDesc));
    texDesc.addressMode[0]   = cudaAddressModeBorder;
    texDesc.addressMode[1]   = cudaAddressModeBorder;
    texDesc.filterMode       = cudaFilterModeLinear;//cudaFilterModePoint //TODO: test filtering for unsigned char
    //texDesc.filterMode     = cudaFilterModePoint;//debug against one of these stupid errors
    texDesc.normalizedCoords = cudaNormalizedTexCoords;//cudaNormalizedTexCoords;
    texDesc.readMode= cudaReadModeNormalizedFloat;//cudaReadModeNormalizedFloat;//read the texture normalized.TODO: maybe as parameter
    if(type == cudaChannelFormatKindFloat){
        texDesc.filterMode       = cudaFilterModeLinear;
        texDesc.readMode = cudaReadModeElementType;//somehow this tends to crash when variable is a float
    }else{
        texDesc.filterMode       = cudaFilterModePoint;
        if(bitDepth == 32){
            texDesc.readMode = cudaReadModeElementType;
        }

    }

    // Create texture object
    cudaCreateTextureObject(&cudaTextureReference, &resDesc, &texDesc, NULL);
    gpuErrchk( cudaPeekAtLastError() );



    //create surface object
    ///TODO: check if this really works
    cudaCreateSurfaceObject(&cudaSurfaceReference,&resDesc);
    gpuErrchk( cudaPeekAtLastError() );


    auto endCudaConfig= chrono::high_resolution_clock::now();

    //this is where this is supposed to be after
    //auto endCuda = chrono::high_resolution_clock::now();

    //this is the time where we change the texture settings(if we do this before binding the texture to cuda this binding fails.
    //and if we dont set these settings we for some reasons can't have bindless)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filterType);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filterType);

    //this in the best case only hides the problem
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    //we want to show the problem to solve it //TODO!!!!!
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

    //float color[] = { 1.0f, 0.0f, 0.0f, 1.0f };
    //glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    //bindless texture magic:
    //glMakeTextureHandleNonResidentARB to make it non resident
    ///it seems that this call needs to be done after
    /// cudaGraphicsGLRegisterImage
    glHandle = glGetTextureHandleARB(glName);

    gfx::GLUtils::checkForOpenGLError("Error while obtaining the GL texture handle");
    //this seems to have to be called once for everytime it has to be rendered?
    /*if(!glIsTextureHandleResidentARB(glHandle)){
        glMakeTextureHandleResidentARB(glHandle);
    }*/




    auto endGlConfig= chrono::high_resolution_clock::now();


    /*
    std::cout << "[gfx::GpuTex2D::GpuTex2D] "
                 "Creation of the GL texture took " <<
                 chrono::duration_cast<chrono::nanoseconds>(endGlTex-start).count() <<
                 "ns" << endl <<  "Cuda Register GL Tex " <<
                  chrono::duration_cast<chrono::nanoseconds>(endCudaGLRegister-endGlTex).count() <<
                 "ns" << endl << "Cuda config took " <<
                 chrono::duration_cast<chrono::nanoseconds>(
                     endCudaConfig -endCudaGLRegister).count() <<
                 "ns" << endl << "GL config took " <<
                 chrono::duration_cast<chrono::nanoseconds>(
                     endGlConfig-endCudaConfig).count() <<
                 "ns" << endl;
    */


    //glDeleteTextures(1,&glName);//DEBUG!!!!
    //cout << "[gfx::GpuTex2D::GpuTex2D] deleting textures is shit" << endl;


    //THIS is a giant debug effort to get rid of errors that are caused
    //by the opengl cuda interoperability.
    /*
    glBindTexture(GL_TEXTURE_3D, 0);
    //cudaGraphicsMapResources(1,&cudaTextureResource,0);
    gpuErrchk( cudaPeekAtLastError() );
    cudaGraphicsUnmapResources(1,&cudaTextureResource,0);
    //cudaGraphicsUnregisterResource(cudaTextureResource);//this throws an unknown error
    //gpuErrchk( cudaPeekAtLastError() );
    glDeleteTextures(1,&glName);
    gfx::GLUtils::checkForOpenGLError("e");

    ///TODO: check if the cudaDestroy functions are really supposed to work this way
    cudaDestroyTextureObject(cudaTextureReference);
    cudaDestroySurfaceObject(cudaSurfaceReference);
    cout << "[gfx::GpuTex2D::~GpuTex2D] destruction of our gpu references and objects for this texture" << endl;
    gpuErrchk( cudaPeekAtLastError() );
    //cout << "am i the only one throwing exceptions" << endl;
    */

}

gfx::GpuTex2D::~GpuTex2D()
{
    gfx::GLUtils::checkForOpenGLError("before destroying the gpu tex");
    overallTexCount--;
    /* helpful debug output
    if(!name.empty()){
        cout << "[gfx::GpuTex2D::~GpuTex2D] destruction of " << name << endl;
    }else{
        cout << "[gfx::GpuTex2D::~GpuTex2D] destruction of our gpu references and objects for this texture" << endl;
    }
    */

    auto start= chrono::high_resolution_clock::now();

    //i think we should make sure that all the textures we want to use are mapped
    //https://github.com/ugovaretto/cuda-opengl-interop/blob/master/texture3d-write/simpleGL3DSurfaceWrite.cpp

    /*
    if(glIsTextureHandleResidentARB(glHandle)){
        glMakeTextureHandleNonResidentARB(glHandle);
    }*/
    uint64_t handle = getGlHandle();
    std::thread::id this_id = this_thread::get_id();
    tokenMutex.lock();
    for(auto token : residentToken){
        thread::id id = token.first;
        if(id != this_id){
            //when deleting the texture in one thread, the
            shared_ptr<bool> sharedToken = token.second;
            function<void()> cleanupTask = [sharedToken,handle](){
                if(!(*sharedToken)){//when the token is set to true then the handle has been cleared already
                    if(glIsTextureHandleResidentARB(handle)){
                        glMakeTextureHandleNonResidentARB(handle);
                    }else{
                        assert(0);//the token should already ensure that the object exists.
                    }

                }
                assert(!gfx::GLUtils::checkForOpenGLError("cleaning up the gpu handle"));
                //this sharedToken is a little unnecessary but it should show that the sharedToken is doing something here
                //it is retained until this function finishes
                //sharedToken.reset();
            };
            garbageCollector->AddToClean(id,cleanupTask);
        }
    }
    residentToken.clear();//clear all token now
    // (this prevents the last resort cleanup method from making the handle non-resident)
    tokenMutex.unlock();


    //cudaGraphicsMapResources(1,&cudaTextureResource,0);
    //gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaPeekAtLastError() );
    cudaDestroyTextureObject(cudaTextureReference);
    gpuErrchk( cudaPeekAtLastError() );
    cudaDestroySurfaceObject(cudaSurfaceReference);
    gpuErrchk( cudaPeekAtLastError() );
    cudaGraphicsUnmapResources(1,&cudaTextureResource,0);
    cudaGraphicsUnregisterResource(cudaTextureResource);//this throws an unknown error
    gpuErrchk( cudaPeekAtLastError() );
    glDeleteTextures(1,&glName);
    gfx::GLUtils::checkForOpenGLError("destroying the gpuTex");

    ///TODO: check if the cudaDestroy functions are really supposed to work this way


    //cout << "am i the only one throwing exceptions" << endl;

    //i hope this forces the driver to free the memory
    glFinish();
    auto end= chrono::high_resolution_clock::now();
    /*
    std::cout << "[gfx::GpuTex2D::~GpuTex2D] "
                 "deletion of the  texture took " <<
                 chrono::duration_cast<chrono::nanoseconds>(end-start).count() <<
                 "ns" << endl;*/

    overallTexListMutex.lock();
    for(size_t i=overallTexList.size()-1;i>=0;i--){
        if(overallTexList[i] == this){
            overallTexList[i] = overallTexList[overallTexList.size()-1];
            overallTexList.pop_back();
            overallTexListMutex.unlock();
            return;
        }
    }
    overallTexListMutex.unlock();
}

void gfx::GpuTex2D::uploadData(void *data)
{
    cudaMemcpyToArray(cudaArrayReference,0,0,data,m_width*m_height*byteCount*channelCount,cudaMemcpyHostToDevice);
}

void gfx::GpuTex2D::uploadData(void *data, int width, int height)
{
    //cout << "[GpuTex2D uploadData]this does not work, use the cuda function!!!!!!!" << endl;
    /*glBindTexture(GL_TEXTURE_2D,glName);
    glTexSubImage2DEXT(GL_TEXTURE_2D,
                        0,//level
                        0,
                        0,
                        width,
                        height,
                        m_glFormat,
                        m_glType,
                        data);
                        */
    uploadData(data,0,0,width,height);

}

void gfx::GpuTex2D::uploadData(void *data, int x, int y, int width, int height)
{
    cudaMemcpy2DToArray(cudaArrayReference,
                        x*byteCount*channelCount,y,//offset of the target
                        data,
                        width*byteCount*channelCount,
                        width*byteCount*channelCount,
                        height,
                        cudaMemcpyHostToDevice);
}

void gfx::GpuTex2D::downloadData(void *data)
{
    downloadData(data,0,0,m_width,m_height);

}

void gfx::GpuTex2D::downloadData(void *data, int x, int y, int width, int height)
{
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    //cout << "[gfx::GpuTex2D::downloadData] THIS IS NOT TESTED YET" << endl;
    cout << "indeed it is not functional!!!!!!! shit!!!!!! << i think the dpitch is not right" << endl;
#endif
    //cout << "channel count (DEBUG) " << channelCount << endl;
    cudaMemcpy2DFromArray(data,
                          byteCount*channelCount*width,//dpitch (not sure about that) //step
                          cudaArrayReference,
                          x*byteCount*channelCount,y,//x in bytes?
                          width*byteCount*channelCount,//definitely in bytes
                          height,
                          cudaMemcpyDeviceToHost);
    gpuErrchk( cudaPeekAtLastError() );
}



void gfx::GpuTex2D::makeResidentInThisThread() {
    thread::id id= this_thread::get_id();
    uint64_t handle = getGlHandle();
    if(!glIsTextureHandleResidentARB(handle)){
        glMakeTextureHandleResidentARB(handle);
        shared_ptr<bool> token = make_shared<bool>(false);//create the token!!!
        tokenMutex.lock();
        residentToken[id] = token;
        tokenMutex.unlock();
        weak_ptr<bool> weakToken = token;
        std::function<void()> lastResortDelete = [weakToken, handle](){
            if(!weakToken.expired()){
                *weakToken.lock() = true;
                //make the handle non resident only if none of the token still exists for this thread
                glMakeTextureHandleNonResidentARB(handle);
                assert(!gfx::GLUtils::checkForOpenGLError("making a handle non resident in thread"));
            }
        };
        garbageCollector->AddToForceCollect(lastResortDelete);

    }

    assert(!gfx::GLUtils::checkForOpenGLError("Error at making the texture resident"));
}


