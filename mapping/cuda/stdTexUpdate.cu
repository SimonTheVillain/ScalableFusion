#include "stdTexUpdate.h"


#include <iostream>
#include <stdio.h>
#include <assert.h>
#include "helper_math.h"
#include "gpuErrchk.h"
#include "surfaceRead.h"
#include "xtionCameraModel.h"
#include "float16_utils.h"



using namespace Eigen;
//using namespace std;



__device__ inline void writeResult(float4 result,cudaSurfaceObject_t surface,int x,int y){

        ushort4 output = float4_2_half4_reinterpret_ushort4_rn(result);
        surf2Dwrite(output,surface,x*sizeof(ushort4),y);


}

__global__ void updateGeomTex_kernel(const cudaSurfaceObject_t geometryInput, //the sensor input adapted by standard deviations
                                     int width,int height, //sensor resolution
                                     gpu::UpdateDescriptor* descriptors,
                                     Eigen::Vector4f camPos,//camera position
                                     Eigen::Matrix4f _pose, // because we want the vertex position relative to the camera
                                     Eigen::Matrix4f proj_pose, //to get the position of the point on the image.
                                     GpuVertex *vertices, Eigen::Vector2f *texPos,
                                     GpuTriangle* triangles,GpuPatchInfo* patchInfos){

    const int k = blockIdx.x;
    gpu::UpdateDescriptor &descriptor = descriptors[k];

    int i=threadIdx.x;
    uint32_t vertexSourceOffset = descriptor.vertex_source_start_ind;
    uint32_t vertexDestOffset = descriptor.vertex_destination_start_ind;

    int absolutePixCount = descriptor.destination.height*descriptor.destination.width;
    while(i<absolutePixCount){
        int x=i%descriptor.destination.width;
        int y=i/descriptor.destination.width;


        //get the pixel coordinate (not normalized)
        int xDest=x + descriptor.destination.x;
        int yDest=y + descriptor.destination.y;

        int xRef=x + descriptor.reference_offset.x;
        int yRef=y + descriptor.reference_offset.y;

        //the propable source coordinate (not normalized)
        float xSource = ((x)*descriptor.source.width)/
                descriptor.destination.width + descriptor.source.x;
        float ySource = ((y)*descriptor.source.height)/
                descriptor.destination.height + descriptor.source.y;


        //DEBUG: Copy the source to destRect:
        if(false){
            float4 surface_k = readFloat4F16(descriptor.source_geometry,
                                  xSource,ySource);

            writeResult(surface_k,descriptor.destination_geometry,xDest,yDest);
            i+=blockDim.x;
            continue;

        }


        //for debugging purposes we just copy the input to the output without any updates
        /*

        float4 surface_k1=make_float4(0,1,2,3);

        surf2Dwrite(surface_k1,descriptor.destinationGeometry,xDest*sizeof(float4),yDest);
        */


        //the old code:


        //read from the reference texture
        float4 ref;
        surf2Dread(&ref,descriptor.destination_references,
                   xRef*sizeof(Vector4f),yRef);



        //TODO: if the  vertex doesn't have a triangle within this patch
        //we want to use a different method. (but for this to work we need to
        //implement code in the vertex update part)

        float bary[3]= {ref.y,ref.z,ref.w};
        int32_t triangleId= *((int*)(&ref.x));

        if(triangleId<0){
            i+=blockDim.x;
            continue;
        }
        GpuTriangle &triangle = triangles[triangleId];


        //i+=blockDim.x;//debug
        //continue;
        //read out the vertices
        Vector4f point(0,0,0,0);
        Vector4f pointUpdated(0,0,0,0);
        for(int j=0;j<3;j++){
            point += vertices[vertexSourceOffset+triangle.indices[j]].p*bary[j];
            pointUpdated += vertices[vertexDestOffset+triangle.indices[j]].p*bary[j];
        }

        //project:
        Vector4f texCoord = proj_pose*point;
        float u=texCoord[0]/texCoord[3];
        float v=texCoord[1]/texCoord[3];

        //read out the current sensor data
        //TODO: this readout probably is shit.....
        float4 sensor = readBilinear(u,v,
                                     geometryInput,
                                     width,height);

        //this should be working better
        sensor = readSensor(u,v,
                            geometryInput,
                            width,height,
                            0.05f);//5cm threshold



        float4 surface_k;
        if(descriptor.destination.width == descriptor.source.width &&
                descriptor.destination.height == descriptor.source.height){
            //if the source and the destRect
            //texture have the same resolution we handle all of this differently
            surface_k = readFloat4F16(descriptor.source_geometry,
                                      xSource,ySource);

        }else{
            surface_k = readBilinear16F(xSource,
                                        ySource,
                                        descriptor.source_geometry,
                                        descriptor.source_size.width,
                                        descriptor.source_size.height);

        }




        //hopefully the border for this readout will be NON
        if(isnan(surface_k.y)){
            //if the old surface data is invalid we just put out the
            //sensor data. (hoping that this is better than nothing)
            //actually this should have a visibility test (TODO)
            writeResult(make_float4(0,sensor.y,sensor.z,sensor.w),
                        descriptor.destination_geometry,
                        xDest,yDest);
            /*
            print("TODO: prepare for 16BIT");
            surf2Dwrite(make_float4(0,sensor.y,sensor.z,sensor.w),
                        descriptor.destinationGeometry,
                        xDest*sizeof(float4),yDest);
                        */
            i+=blockDim.x;
            continue;
        }
        if(isnan(sensor.y)){
            //the sensor doesn't have any valid values
            //keep the old surface parameter
            writeResult(surface_k,descriptor.destination_geometry,xDest,yDest);
            /*
            print("TODO: prepare for 16BIT");
            surf2Dwrite(surface_k,descriptor.destinationGeometry,xDest*sizeof(float4),yDest);
            */
            i+=blockDim.x;
            continue;
        }
        //transform the coordinates relative to camera
        Vector4f posCamFrame = _pose*point;

        Vector4f posUpCamFrame = _pose*pointUpdated;
        //depth test against sensor input
        float d = posCamFrame[2];
        float dUp=posUpCamFrame[2];

        //TODO: make this threshold dependant on something
        float threshold = 0.05f;//+- 5 cm? maybe this is not the right one.

        threshold = xtionStdToThresholdSeg(max(surface_k.z,sensor.z));
        //(like the standard deviation of the sensor)
        if(abs(d-sensor.x) > threshold){
            //if the surface is hidden by some other surface.
            //keep the old surface parameter
            //print("TODO: prepare for 16BIT");
            //surf2Dwrite(surface_k,descriptor.destinationGeometry,xDest*sizeof(float4),yDest);
            writeResult(surface_k,descriptor.destination_geometry,xDest,yDest);
            i+=blockDim.x;
            continue;
        }



        float4 surface_k1 = calcSurfaceUpdate(surface_k,sensor, //the vector of sensor data and of what is on the surface
                                              d, dUp);
        writeResult(surface_k1,descriptor.destination_geometry,xDest,yDest);



        i+=blockDim.x;
    }




    //done!!!!!!


    //TO think
    //NO!!!!!it is not! we can't do the update step upfront.
    //but for each point we can store the position of how it was before and how it will be afterwards.....
    //so we can have the information of each point before and after the update.
    //thus we can calculate for each pixel the distane between the old and the new surface.
    //ONE QUESTION REMAINS: SHOULD THIS BE DONE IN ONE OR TWO KERNEL CALLS????
    //for 2 kernel calls we would need a buffer to store the intermediate results

    //For 1 kernel call we have the problem of some of the vertices not being part of the
    //patch. (actually this problem applies for both approaches)

    //We need 2 calls + one copy of the vertex buffer!!!!:
    //1) update the new texture + update the second vertex buffer
    //2) update the change in distance between the geometry gathered by the second
    //   vertex buffer compared to the first write the geometry from the second buffer
    //   to the first.


    //TE DO THIS:!!!!!!
    //without only one kernel call:
    // First: we create the updated version of the vertices in the shared memory
    // second:
    // for each pixel we read the 3 neighboring vertices.
    // we create the updated version of the vertex + keep the old one,
    // according to this we update the texture. ( this means calculating the update
    // according to the old geometry and later updating it with the new one)
    // After writing out the texture we write out the vertices.



}

void updateGeomTexturesOfPatches(const cudaSurfaceObject_t geometryInput, //the sensor input adapted by standard deviations
                   int width,int height, //sensor resolution
                   const std::vector<gpu::UpdateDescriptor> &descriptors,
                   Eigen::Vector4f camPos,
                   Eigen::Matrix4f _pose, // because we want the vertex position relative to the camera
                   Eigen::Matrix4f proj_pose, //to get the position of the point on the image.
                   GpuVertex *vertices, Eigen::Vector2f *texPos,
                   GpuTriangle* triangles,GpuPatchInfo* patchInfos){ //pointer to the geometric data

    if(descriptors.size()==0){
        return;
    }
    dim3 block(1024);
    dim3 grid(descriptors.size());
    gpu::UpdateDescriptor *descs;
    cudaMalloc(&descs,descriptors.size()*sizeof(gpu::UpdateDescriptor));
    cudaMemcpy(descs,&descriptors[0],descriptors.size()*sizeof(gpu::UpdateDescriptor),cudaMemcpyHostToDevice);//copy it to the device

    gpuErrchk( cudaPeekAtLastError() );

    updateGeomTex_kernel<<< grid,block >>>(
                               geometryInput, //the sensor input adapted by standard deviations
                               width,height, //sensor resolution
                               descs,
                               camPos,
                               _pose, // because we want the vertex position relative to the camera
                               proj_pose, //to get the position of the point on the image.
                               vertices, texPos,
                               triangles,patchInfos);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
    cudaFree(descs);


}


__global__ void dilateLookupTextures_kernel(const DilationDescriptor* descriptors){
    unsigned int k =blockIdx.x;
    const DilationDescriptor &descriptor = descriptors[k];
    const unsigned int requiredPixel=descriptor.height*descriptor.width;
    unsigned int i = threadIdx.x;



    const int2 direct[] = {make_int2(-1,0), make_int2(1,0), make_int2(0,-1), make_int2(0,1)};
    const int2 indirect[] = {make_int2(-1,-1), make_int2(1,1), make_int2(1,-1), make_int2(-1,1)};
    while(i<requiredPixel){

        int x=i%descriptor.width + descriptor.x;//derive the target pixel coordinates from k
        int y=i/descriptor.width + descriptor.y;
        float4 center;
        surf2Dread(&center,descriptor.target,x*4*4,y);
        int index = *((int*)(&center.x));

        if(index != -1){//if the triangle index already is valid we don't do this
            i+=blockDim.x;
            continue;
        }

        bool set=false;
        //otherwise we search at the neighbouring locations
        for(int j=0;j<4;j++){
            int x2=x+direct[j].x;
            int y2=y+direct[j].y;
            if( x2 < descriptor.x || y2 < descriptor.y ||
                    x2 >= (descriptor.width + descriptor.x) ||
                    y2 >= (descriptor.height + descriptor.y) ){
                continue;
            }
            float4 neighbour;
            surf2Dread(&neighbour,descriptor.target,x2*4*4,y2);
            index=*((int*)(&neighbour.x));
            if(index == -1){
                continue;
            }
            surf2Dwrite(neighbour,descriptor.target,x*4*4,y);
            set=true;
            break;

        }
        if(set){
            i+=blockDim.x;
            continue;
        }
        //if this didn't yield results we do it diagonally
        for(int j=0;j<4;j++){
            int x2=x+indirect[j].x;
            int y2=y+indirect[j].y;
            if( x2 < descriptor.x || y2 < descriptor.y ||
                                x2 >= (descriptor.width + descriptor.x) ||
                                y2 >= (descriptor.height + descriptor.y) ){
                continue;
            }
            float4 neighbour;
            surf2Dread(&neighbour,descriptor.target,x2*4*4,y2);
            index=*((int*)(&neighbour.x));
            if(index == -1){
                continue;
            }
            surf2Dwrite(neighbour,descriptor.target,x*4*4,y);
            set=true;
            break;
        }
        //do the next few pixel
        i+=blockDim.x;
    }

}

void dilateLookupTextures(const std::vector<DilationDescriptor> &descriptors){
    if(descriptors.size()==0){
        return;
    }
    dim3 block(1024);
    dim3 grid(descriptors.size());
    DilationDescriptor *descs;
    cudaMalloc(&descs,descriptors.size()*sizeof(DilationDescriptor));
    cudaMemcpy(descs,&descriptors[0],descriptors.size()*sizeof(DilationDescriptor),cudaMemcpyHostToDevice);//copy it to the device

    dilateLookupTextures_kernel<<<grid,block>>>(descs);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
    cudaFree(descs);
}


__global__ void stdTexInit_kernel(const cudaTextureObject_t input,const InitDescriptor *descriptors,
                     Eigen::Matrix4f proj_pose,
                     GpuVertex *vertices,Eigen::Vector2f *texPos,
                     GpuTriangle *triangles,GpuPatchInfo *patchInfos){

    unsigned int k = blockIdx.x;
    const InitDescriptor &descriptor = descriptors[k];
    const unsigned int requiredPixel=descriptor.height*descriptor.width;
    unsigned int i = threadIdx.x;

    while(i<requiredPixel){
        int xRef=i%descriptor.width + descriptor.refOffset.x;//derive the target pixel coordinates from k
        int yRef=i/descriptor.width + descriptor.refOffset.y;
        int xOut=i%descriptor.width + descriptor.outOffset.x;//derive the target pixel coordinates from k
        int yOut=i/descriptor.width + descriptor.outOffset.y;



        //this puts the whole image to
        //float4 geom = tex2D<float4>(descriptor.,u,v);
        float4 ref;
        surf2Dread(&ref,descriptor.referenceTexture,xRef*4*4,yRef);
        float bary[3]= {ref.y,ref.z,ref.w};
        int32_t triangleId= *((int*)(&ref.x));//reinterpret the float as integer.... i really hope this works


        //DEBUG:
        if(false){// this is to update
            float4 blabla;
            if(triangleId<0 && true){
                blabla = make_float4(1,0,0,1);
            }else{
                blabla = make_float4(0,0,1,1);
                if(xOut%8 <4 && yOut%8>4){
                    //checkerboard pattern
                    //blabla = make_float4(0,1,0,1);
                }
                //blabla = make_float4(ref.y,ref.z,ref.w,1);

            }
            ushort4 bla = make_ushort4(__float2half_rn(blabla.x),
                                        __float2half_rn(blabla.y),
                                        __float2half_rn(blabla.z),
                                        __float2half_rn(blabla.w));

            surf2Dwrite(bla,descriptor.output,xOut*sizeof(ushort4),yOut);


            i+=blockDim.x;
            continue;
        }

        if(triangleId<0){
            float4 color = make_float4(NAN,NAN,NAN,NAN);

            ushort4 data = float4_2_half4_reinterpret_ushort4_rn(color);
            /*
                    make_ushort4(__float2half_rn(color.x),
                                        __float2half_rn(color.y),
                                        __float2half_rn(color.z),
                                        __float2half_rn(color.w));
                                        */
            surf2Dwrite(data,descriptor.output,xOut*sizeof(ushort4),yOut);//write some debug value to the output (seems to work)

            i+=blockDim.x;
            continue;
        }
        GpuTriangle triangle = triangles[triangleId];
        Vector4f point(0,0,0,0);
        for(int j=0;j<3;j++){
            GpuPatchInfo & info = patchInfos[triangle.patchInfoInds[j]];
            int index = info.vertexSourceStartInd +
                    triangle.indices[j];
            point += vertices[index].p*bary[j];
        }
        //printf("%f",point[3]);
        //point = _pose*point;//transform the coordinate system //in the first frame pose anyway is Identity
        Vector4f texCoord = proj_pose*point;//project (Non-normalized coordinates)
        //TODO: depth test and reading of correct color/geom values
        //are these normal values
        float u=texCoord[0]/texCoord[3];
        float v=texCoord[1]/texCoord[3];

        //look at applyNewColorData in scaleableMapTexturing, there the creation of texture
        //coordinates works well. why isn't it working here?

        //if this is right do the projection and read the according values. (also do a depth test)

        //geom=make_float4(1.0f,0.0f,0.0f,1.0f);

        float4 color = make_float4(point[0],point[1],point[2],point[3]);
        color = make_float4(u,v,0.0f,1.0f);
        //color = make_float4(1.0f,0.0f,0.0f,1.0f);//DEBUGGIDIBUG



        float4 sensor = tex2D<float4>(input,u,v);

        /*
        if(k==0){
            //printf("u, v, sensor %f, %f %f \n",u,v,sensor.y);
            if(i==0){
                printf("%f %f %f %f\n"
                       "%f %f %f %f\n"
                       "%f %f %f %f\n"
                       "%f %f %f %f\n",
                       proj_pose(0,0),proj_pose(1,0),proj_pose(2,0),proj_pose(3,0),
                       proj_pose(0,1),proj_pose(1,1),proj_pose(2,1),proj_pose(3,1),
                       proj_pose(0,2),proj_pose(1,2),proj_pose(2,2),proj_pose(3,2),
                       proj_pose(0,3),proj_pose(1,3),proj_pose(2,3),proj_pose(3,3));
            }
        }
         */

        //color=make_float4(0.5f*point[3],0.0f,0.0f,1.0f);//test if the coordinates really are barycentric
        //surf2Dwrite(color,descriptor.output,x*4*4,y);

        //TODO: keep the standard deviations 1/1 but set the depth offset to zero, when there are NANs we
        //set the values to the closest possible value
        color.x=0;//depth offset is zero,
        color.y=sensor.y; // here we have to check if it is NAN... this would not be too awesome
        color.z=sensor.z; // same here, i don't know what to do tough


        //debug:
        /*
        color.x=u;
        color.y=v;
        color.z=0;
        if(u<0 || v < 0){
            color.z=1;
        }
        color.w=1;
         */
        //color.y=1;//print yellow everywhere
        //color.z=1;
        //color = make_float4(point[0],point[1],point[2],point[3]);


        ushort4 output = float4_2_half4_reinterpret_ushort4_rn(color);
        /*
                make_ushort4(hackyReinterpretCast(__float2half_rn(color.x)),
                                      hackyReinterpretCast(__float2half_rn(color.y)),
                                      hackyReinterpretCast(__float2half_rn(color.z)),
                                      hackyReinterpretCast(__float2half_rn(color.w)));
        output = float4_2_half4_reinterpret_ushort4_rn(color);
         */

        /*
        if(k==0 && i==0){

            printf("soooooo. lets find this fucker%#10x\n",hackyReinterpretCast(0.1f));

            printf("soooooo. lets find this fucker%#10x\n",hackyReinterpretCast(__float2half(0.1f)));//__half2float(0.1f));
        }*/
        //__half half = float2half
        //printf("what is going on? %d\n",(int)hackyReinterpretCast(__float2half(color.x)));//reinterpret_cast<ushort>(__float2half_rn(color.y)));
        surf2Dwrite(output,descriptor.output,xOut*sizeof(ushort4),yOut);



        //do the next block
        i+=blockDim.x;
    }

}

void stdTexInit(const cudaTextureObject_t input,const std::vector<InitDescriptor> &descriptors,
                Eigen::Matrix4f proj_pose,
                GpuVertex *vertices,Eigen::Vector2f *texPos,
                GpuTriangle *triangles,GpuPatchInfo *patchInfos){

    InitDescriptor *descs=NULL;
    //create a buffer for all the commands:
    //TODO: DELETE THIS DEBUG SHIT!!!!!
    for(size_t i = 0;i<descriptors.size();i++){
        InitDescriptor desc = descriptors[i];
        /*
        if(desc.x+desc.width >= 1024 || desc.y+desc.height >=1024){
            assert(0);
        }
        cout << "descriptor" << i << endl <<
                "x " << descriptors[i].x << " y " << descriptors[i].y << endl <<
                " width " << descriptors[i].width << " height "<< descriptors[i].height << endl <<
                " output " << descriptors[i].output << " reference " << descriptors[i].referenceTexture <<
                endl;
        */

    }
    cudaMalloc(&descs,descriptors.size()*sizeof(InitDescriptor));



    gpuErrchk( cudaPeekAtLastError() );
    cudaMemcpy(descs,&descriptors[0],descriptors.size()*sizeof(InitDescriptor),cudaMemcpyHostToDevice);//copy it to the device

    gpuErrchk( cudaPeekAtLastError() );
    //start the kernel:
    dim3 block(1024);//smaller is better for smaller images. how is it with bigger images?
    //the ideal way would be to start blocks of different sizes. (or maybe use CUDAs dynamic parallelism)
    //for higher resolution images 1024 or even higher threadcounts might be useful

    dim3 grid(descriptors.size());
    if(grid.x!=0){
        stdTexInit_kernel<<<grid,block>>>(input,descs,
                                          proj_pose,
                                          vertices,texPos,
                                          triangles,patchInfos);
    }
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
    //AS LONG AS WE ARE NOT REUPLOADING AND CREATING NEW REFERENCE TEXTURES TO THE GEOMETRY
    //THIS JUST FAILS (HOPEFULLY,  BECAUSE THE REFERENCES ARE POINTING TO NOWHERE)



    //delete the buffer with the commands
    cudaFree(descs);

}




//this was something i did for debugging purposes. it is supposed to shift the
__global__ void shiftVerticesALittle_kernel(GpuVertex *vertices,size_t from,size_t to){

    size_t element = threadIdx.x + blockIdx.x*blockDim.x + from;
    if(element >= to){
        return;//return if we are exceeding the buffer
    }
    Eigen::Vector4f offset(10.1,0,0,0);
    vertices[element].p += offset;//that should be it
}

void shiftVerticesALittle(GpuVertex *vertices,size_t from,size_t to){
    size_t vertexCount = to-from;
    dim3 block(256);
    dim3 grid(vertexCount/block.x + 1);

    shiftVerticesALittle_kernel<<<block,grid>>>(vertices,from,to);
    cudaDeviceSynchronize();


}
