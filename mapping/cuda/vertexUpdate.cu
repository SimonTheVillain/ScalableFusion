#include "vertexUpdate.h"


#include <iostream>
#include <stdio.h>
#include <assert.h>
#include "helper_math.h"
#include "surfaceRead.h"
#include "xtionCameraModel.h"
#include "geomUpdate.h"
#include "float16_utils.h"

using namespace Eigen;
//using namespace std; //note never use std namespace in cu file


__global__ void vertexUpdate_kernel(const cudaSurfaceObject_t geometryInput, //the sensor input adapted by standard deviations
                                     int sensorWidth,int sensorHeight, //sensor resolution
                                     gpu::UpdateDescriptor* descriptors,
                                     Eigen::Vector4f camPos,//camera position
                                     Eigen::Matrix4f _pose, // because we want the vertex position relative to the camera
                                     Eigen::Matrix4f proj_pose, //to get the position of the point on the image.
                                     GpuVertex *vertices, Eigen::Vector2f *texPos,
                                     GpuTriangle* triangles,GpuPatchInfo* patchInfos){
    uint32_t k = blockIdx.x;
    gpu::UpdateDescriptor &descriptor = descriptors[k];
    GpuPatchInfo &info = patchInfos[descriptor.patch_info_slot];
    uint32_t i = threadIdx.x;
    uint32_t vertexSourceOffset = descriptor.vertex_source_start_ind;
    uint32_t vertexDestOffset = descriptor.vertex_destination_start_ind;


    //TODO: remove this as this is a attempt to check if the bound texture is valid!!!
    /*
    int abs_pix_count = descriptor.source.width * descriptor.source.height;



    while(i < abs_pix_count){
        int y = i/descriptor.source.width + descriptor.source.y;
        int x = i%descriptor.source.width + descriptor.source.x;
        //x=0;
        //y=0;
        printf("%d %d \n",x,y);
        ushort4 surfaceData = float4_2_half4_reinterpret_ushort4_rn(make_float4(1,0,0,1));//its happening here!!!
        surf2Dwrite(surfaceData,descriptor.sourceGeometry,x*sizeof(ushort4),y);
        //surf2Dwrite(make_float4(1,1,1,1),geometryInput,x*sizeof(float4),y);
        i+=blockDim.x;

    }
    abs_pix_count = 640*480;
    while(i < abs_pix_count){
        int y = i/640;
        int x = i%640;
        x=x;
        y=y;
        surf2Dwrite(make_float4(1,1,1,1),geometryInput,x*sizeof(float4),y);
        i+=blockDim.x;

    }*/

    //printf("%d vertexSourceOffset \n",vertexSourceOffset);
    //return;
    uint32_t texPosOffset = info.std_texture.tex_coord_start_ind;//*nrTexPosPerPatch;
    while(i<descriptor.vertex_count){
        GpuVertex &vertIn = vertices[vertexSourceOffset+i];
        GpuVertex &vertOut = vertices[vertexDestOffset+i];

        //most of the vertex is the same anyway (therefore we just copy it)
        vertOut=vertIn;
        //debug, dont update anything just leave it as it is
        //printf("%d \n",i);
        //i+=blockDim.x;
        //continue;

        Vector4f pos = vertIn.p;
        pos[3]=1; //just to be sure;


        Eigen::Vector4f posCamFrame = _pose*pos;
        Vector4f posOnSensor = proj_pose*pos;

        float u=posOnSensor[0]/posOnSensor[3];
        float v=posOnSensor[1]/posOnSensor[3];

        //debug: remove this
        {

            //printf("uv: %f %f \n",u,v);
            //surf2Dwrite(surfaceData,descriptor.destinationGeometry,int(uv.x)*sizeof(ushort4),int(uv.y));//TODO: reinsert this

            //i+=blockDim.x;
            //continue;
        }


        //TODO: generate threshold here:

        float4 sensor =
                readSensor(u,v,
                                 geometryInput,
                                 sensorWidth,sensorHeight,
                           0.05);//threshold =0.1



        //debug: remove this
        //printf("uv: %f %f sensor value: %f %f %f %f \n",u,v,sensor.x,sensor.y,sensor.z,sensor.w);
        //i+=blockDim.x;
        //continue;

        //extract the depth from the readout data
        float d = posCamFrame[2];
        float d_s = sensor.x;


        //get the camera position relative to our vertex
        Vector4f toCamera = camPos-pos;
        Vector3f toCamN = toCamera.block<3,1>(0,0)*1.0f/toCamera.block<3,1>(0,0).norm();
        Vector4f front(toCamN[0],toCamN[1],toCamN[2],0);
        //Vector3f front = make_float3(toCamN[0],toCamN[1],toCamN[2]);

        //read the current value of the texture
        Vector2f texCoord = texPos[texPosOffset + vertIn.tex_ind_in_main_patch];


        //this tex coordinate still has to be adapted for the texture atlas
        float2 texAtlasCoord = make_float2(
                    texCoord[0]*descriptor.source_n.width + descriptor.source_n.x,
                    texCoord[1]*descriptor.source_n.height + descriptor.source_n.y);
        //if(k==10)
        //    printf("texAtlasCoord %f, %f \n",texAtlasCoord.x,texAtlasCoord.y);


        //TODO: we need a function that explicitely handles reading textures from this type of texture

        //texture atlas
        //PLUS: if the point we want to read out has no valid pixel (not set for some reason)
        //we set the pixel at this position. (in the next readout we might be lucky)
        //reading the source geometry
        /*float4 surface_k=
                readBilinearGLStyle(texAtlasCoord.x,texAtlasCoord.y,
                                    descriptor.sourceGeometry,
                                    descriptor.sourceSize.width,descriptor.sourceSize.height);
        */
        float2 uv = unnormalizeTexCoords(make_float2(texCoord[0],texCoord[1]),
                                    descriptor.source);
        float4 surface_k;
        {
            /*float2 uv = unnormalizeTexCoords(make_float2(texCoord[0],texCoord[1]),
                                        descriptor.source);
            */
            vertOut.n=Eigen::Vector3f(0,0,1);
            __half surfaceData[4];
            surf2Dread((ushort4*)surfaceData,descriptor.source_geometry,int(uv.x)*sizeof(ushort4),int(uv.y));
            surface_k = make_float4(__half2float(surfaceData[0]),
                                    __half2float(surfaceData[1]),
                                    __half2float(surfaceData[2]),
                                    __half2float(surfaceData[3]));
            //debug: i really think it is the readout here!!!!!

        }

        //very desperate debug measure
        //printf("surface_k %f %f %f %f \n",surface_k.x,surface_k.y,surface_k.z,surface_k.w);
        vertOut.n=Eigen::Vector3f(0,1,0);
        if(min(uv.x,uv.y) < 0 ||
            max(uv.x,uv.y) > 1023){
            printf("uv %f %f\n",int(uv.x),int(uv.y));
        }
        //vertOut.n=Eigen::Vector3f(surface_k.x,surface_k.y,surface_k.z);
        //ushort4 surfaceData = float4_2_half4_reinterpret_ushort4_rn(surface_k);//its happening here!!!
        //surf2Dwrite(surfaceData,descriptor.sourceGeometry,int(uv.x)*sizeof(ushort4),int(uv.y));
        //i+=blockDim.x;
        //continue; //DEBUG 113:not until here
        if(isnan(surface_k.w)){//actually this should be 1
            //if this surface element is invalid we store the sensor input at the position that is lacking

            float4 update = make_float4(0,sensor.y,sensor.z,2);//mark the lonesome point as being a lonesome point (2)

            ushort4 surfaceData = float4_2_half4_reinterpret_ushort4_rn(update);

            //TODO: reinsert this
            surf2Dwrite(surfaceData,descriptor.source_geometry,int(uv.x)*sizeof(ushort4),int(uv.y));

            //TODO: Important!!! Make sure that also the reference is set properly (otherwise we will not see an update)

            if(int(uv.x)<descriptor.source.x ||
                    int(uv.x)-descriptor.source.x >=descriptor.source.width ||
                    int(uv.y)<descriptor.source.y ||
                    int(uv.y)-descriptor.source.y >=descriptor.source.height){
                vertOut.n=Eigen::Vector3f(0,1,0);
            }else{
                vertOut.n=Eigen::Vector3f(1,0,0);
            }
            //obviously this vertex had no correct pixel yet therefore we set a pixel in there
            i+=blockDim.x;
            continue;
        }
        //i+=blockDim.x;
        //continue; //DEBUG 113: but until here!!

        float threshold = xtionStdToThresholdSeg(max(sensor.z,surface_k.z));//the second one is the surface
        if( abs(d-d_s) > threshold ){
            //TODO: make these thresholds more dependant on the noise. (and or the normal)
            //if the depth is bigger than the depth of the sensor + some threshold
            //we do not continue due to being occluded.
            i+=blockDim.x;
            continue;

        }


        //debug
        //printf("surface_k %f %f %f %f \n",surface_k.x,surface_k.y,surface_k.z,surface_k.w);
        //i+=blockDim.x;
        //continue;

#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
        if(k==10)//i have the feeling that reading out from the surface is not working
            printf("surface parameters at vertex, %f, %f, %f, %f for index %d and tex coordinate %f, %f\n",
                   surface_k.x,surface_k.y,surface_k.z, surface_k.w,vertIn.texIndInMainPatch,
                   texAtlasCoord.x,texAtlasCoord.y);//texCoord[0],texCoord[1]);
#endif

        float dev_s=d-d_s;//deviation of the sensor(d) from the surface (d_s)

        //the minimum possible standard deviation
        float s_m = min(sensor.z,surface_k.z);
        float s_k = surface_k.y;
        float s_s = sensor.y;


        //TODO: it actually would increase the numeric stability if the
        //texture would not store s_s but s_s_ so the distance to the minimal
        //possible standard deviation to prevent (numeric) cancellation.
        //standard deviation to the lower bound of standard deviation
        float s_s_ = s_s-s_m;
        float s_k_ = s_k-s_m;


        float dev_k1=dev_s*s_k_/(s_k_+s_s_);
        if(isnan(dev_k1) || isinf(dev_k1)){
            i+=blockDim.x;
            continue;
        }




        //only the position changes
        vertOut.p = vertIn.p + dev_k1*front;


        //debug
        //printf("surface_k %f %f %f %f \n",surface_k.x,surface_k.y,surface_k.z,surface_k.w);
        //i+=blockDim.x;
        //continue;
        //if we are points that do not really belong to a triangle
        //we store the standard deviation within one pixel
        //the update also has to be done in here
        if(surface_k.w==2.0f){
            //TODO: think about depth dependant thresholding????
            float dUp= d-dev_k1;
            threshold = xtionStdToThresholdSeg(sensor.z);
            if(abs(dUp) < threshold){
                break;
            }
            //update the standard deviation in the same way as it would be done
            float4 update = calcSurfaceUpdate(surface_k,sensor,
                                              d,dUp);
            //float4 update = make_float4(0,sensor.y,sensor.z,2);//mark the lonesome point as being a lonesome point (2)

            update.x=0;
            ushort4 surfaceData = float4_2_half4_reinterpret_ushort4_rn(update);
            surf2Dwrite(surfaceData,descriptor.destination_geometry,int(uv.x)*sizeof(ushort4),int(uv.y));

            //TODO: make sure to at some point update the normals by sensible means
            vertOut.n=Eigen::Vector3f(0,1,0);
        }


        if(int(uv.x)<descriptor.source.x ||
                int(uv.x)-descriptor.source.x >=descriptor.source.width ||
                int(uv.y)<descriptor.source.y ||
                int(uv.y)-descriptor.source.y >=descriptor.source.height){
            //set the normal to a debug color?
            vertOut.n=Eigen::Vector3f(1,1,1);
        }

        //just in case we have more vertices than threads:
        i+=blockDim.x;
    }


}

__global__ void calcCenter_kernel(gpu::GeometryUpdate::CalcCenterTask* tasks,
                                  Eigen::Vector4f* centers){
    extern __shared__ Vector4f accu[];
    //__shared__ Vector4f accu[512];
    //Vector4f accu[256];
    uint32_t k = blockIdx.x;
    gpu::GeometryUpdate::CalcCenterTask task = tasks[k];

    Vector3f accumulated(0,0,0);
    uint32_t i = threadIdx.x;
    //uint32_t count = 0;
    while(i<task.count){
        //count++;
        accumulated += task.vertices[i].p.block<3,1>(0,0);
        /*printf("vertex: %f %f %f %f \n",task.vertices[i].p[0],
                task.vertices[i].p[1],
                task.vertices[i].p[2],
                task.vertices[i].p[3]);
                */
        i += blockDim.x;
    }

    int tid = threadIdx.x;
    accu[tid].block<3,1>(0,0) = accumulated;
    __syncthreads();

    for(int s= blockDim.x/2;s>0;s>>=1){
        if(tid>=s){
            //return;//TODO: maybe add this back in?!
            break;//just so we end up at syncthreads faster
        }
        if(tid<s){
            accu[tid].block<3,1>(0,0) += accu[tid+s].block<3,1>(0,0);
        }
        __syncthreads();
    }





    if(tid == 0){
        accu[0] = accu[0]*(1.0f/float(task.count));
    }
    //return;//DEBUG
    __syncthreads();
    Vector3f center = accu[0].block<3,1>(0,0);



    //we can calc radii in the same kernel call!!!!!!

    float maxSqrDist = 0;
    int maxInd=-1;
    i = threadIdx.x;
    while(i<task.count){
        Vector3f pos = task.vertices[i].p.block<3,1>(0,0);
        Vector3f delta = pos-center;
        float sqrDistance = delta.dot(delta);
        //printf("delta = %f %f %f sqrDistance = %f\n",delta[0],delta[1],delta[2],sqrDistance);
        if(sqrDistance>maxSqrDist){
            maxSqrDist = sqrDistance;
            maxInd = i;
        }
        //accumulated += task.vertices[i].p;
        i += blockDim.x;
    }


    struct Outermost{
        int index;
        float sqrDistance;
    };
    //recasting the shared memory:
    Outermost *accuR = (Outermost*)accu;

    //SOMEHOW reinterpret cast is not working
    //float test1[] = {1, 2, 3};
    //int *test2 = std::reinterpret_cast<int*>(test1);
    //int *test = std::reinterpret_cast<int*>(accu);


    __syncthreads();//syncthreads here because we are using the same shared memory
    //accu[tid] = accumulated;
    accuR[tid].index = maxInd;
    accuR[tid].sqrDistance = maxSqrDist;
    __syncthreads();
    //reduction
    for(int s= blockDim.x/2;s>0;s>>=1){
        if(tid>=s){
            return;//TODO: maybe add this back in?!
            break;//just so we end up at syncthreads faster
        }
        if(tid<s){
            if(accuR[tid].sqrDistance < accuR[tid+1].sqrDistance){
                accuR[tid].sqrDistance = accuR[tid+1].sqrDistance;
                accuR[tid].index = accuR[tid+1].index;
            }
        }
        __syncthreads();
    }



    if(tid==0){
        centers[k] = Vector4f(center[0],center[1],center[2],accuR[0].sqrDistance);
    }

}

__global__ void calcRadius_kernel(gpu::GeometryUpdate::CalcCenterTask* tasks, Eigen::Vector4f* centersRadii){

    /*
    struct Outermost{
        int index;
        float sqrDistance;
    };
    extern __shared__ Outermost outermosts[];

    uint32_t k = blockidx.x;
    GeometryUpdate::CalcCenterTask task = tasks[k];
    Vector4f center = centersRadii[k];
    center[k][3]=0;

    float maxSqrDist=0;
    int maxIndex=-1;

    while(i<task.size){
        Vector4f delta = center - task.vertices[i].p;

        i += blockDim.x;
    }
*/


}





