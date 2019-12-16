#include <cuda/tex_patch_init.h>

#include <cuda/gpu_errchk.h>

/**
 * right now we do this for a 4 channel floatt texture, in future this has to work with multiple texture types
 * + maybe extend this for streams
 */
__global__ 
void copyToTexPatches_kernel(const cudaTextureObject_t input,
                              const CopyDescriptor *copies) {
	unsigned int k = blockIdx.x;
	const CopyDescriptor &copy = copies[k];
	const unsigned int required_pixel = copy.target_height * copy.target_width;
	unsigned int i = threadIdx.x;
	while(i<required_pixel) {
		int x = i % copy.target_width;
		int y = i / copy.target_width;

		///TODO: i am not sure about the 0.5f ... as usual
		/// TODO: think about it I THOUGHT AND I THINK THIS IS RIGHT
		/// BUT MANY OF THESE CALCULATIONS COULD BE OPTIMIZED!!!!!
		float u = copy.x + copy.width * ((float) x + 0.5f) / 
		          (float) copy.target_width; // maybe we should store the inverse of width/height
		float v = copy.y + copy.height * ((float) y + 0.5f) / 
		          (float) copy.target_height;

		//now shift x and y to the according place in the texture atlas.
		x= x + copy.target_x;
		y= y + copy.target_y;

		//i feel the coordinates are unfortunately not normalized (maybe we should set the texture accordingly
		float4 color = tex2D<float4>(input, u, v);

		uchar4 color_bytes = make_uchar4(color.x * 255, color.y * 255, 
		                                 color.z * 255, color.w * 255);
		surf2Dwrite(color_bytes, copy.target, x * 4, y);

		//do the next block
		i += blockDim.x;
	}
}

void copyToTexPatches(const cudaTextureObject_t input,
                       const std::vector<CopyDescriptor> &descriptors) {
	dim3 block(256);//smaller is better for smaller images. how is it with bigger images?
	//the ideal way would be to start blocks of different sizes. (or maybe use CUDAs dynamic parallelism)
	//for higher resolution images 1024 or even higher threadcounts might be useful
	dim3 grid(descriptors.size());
	if(grid.x == 0) {
		return;
	}

	//create and copy the descriptors to a gpu buffer
	CopyDescriptor *descs;
	cudaMalloc(&descs, descriptors.size() * sizeof(CopyDescriptor));
	cudaMemcpy(descs, &descriptors[0], 
	           descriptors.size() * sizeof(CopyDescriptor), 
	           cudaMemcpyHostToDevice);

	copyToTexPatches_kernel<<<grid, block>>>(input, descs);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
	cudaFree(descs);
}
