#ifndef FILE_TEX_PATCH_INIT_H
#define FILE_TEX_PATCH_INIT_H
/**
 * The code that helps us copying the texture into a lot of tiny patches
 */

#include <vector>

struct CopyDescriptor {

	cudaSurfaceObject_t output;

	/**
	 * Discussion whats doable with a surface object and what is not:
	 * CUDA arrays are opaque memory layouts optimized for texture fetching.
	 * They are one dimensional, two dimensional, or three-dimensional and composed of elements,
	 * each of which has 1, 2 or 4 components that may be signed or unsigned 8-, 16-, or 32-bit integers,
	 * 16-bit floats, or 32-bit floats. CUDA arrays are only accessible by kernels through texture
	 * fetching as described in Texture Memory or surface reading and writing as described in Surface Memory.
	 * SO: we have to have a surface object.... lets see about that...
	 * ALSO: reading and writing to a surface from the same thread is baaaad when only doing one surface at only one
	 * workgroup it might be OK, but doing this for further workgroups this might be a pretty bad idea:
	 * https://devtalk.nvidia.com/default/topic/686211/using-surfaces-in-a-stack-implementation/
	 */
	//how to get the surface object?
	//http://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#cubemap-surfaces

	//the position from which we read the data
	float x;
	float y;
	//the size of the rect from which we read the data
	float width;
	float height;
	//the size of the target image (maybe we can read this from the cudaSurfaceObject_t)
	int target_x;
	int target_y;
	int target_width;
	int target_height;
};

//create an array of this copy descriptor for all the copies we want to make and start an kernel
void copyToTinyPatches(const cudaTextureObject_t input,
                       const std::vector<CopyDescriptor> &descriptors);

#endif
