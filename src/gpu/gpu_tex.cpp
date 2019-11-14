#include <gpu/gpu_tex.h>

#include <iostream>
#include <cstring>
#include <assert.h>

#include <cuda.h>
#include <cuda_gl_interop.h>

#include "gpu/gl_utils.h"
#include "gpu/garbage_collector.h"

using namespace std;

inline void gpuAssert(cudaError_t code, const char *file, int line, 
                      bool abort = true) {
	if (code != cudaSuccess) {
		fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, 
		        line);
		if(abort) 
			assert(0);
	}
}
#define gpuErrchk(ans) gpuAssert((ans), __FILE__, __LINE__);

int gfx::GpuTex2D::overall_tex_count_ = 0;
int gfx::GpuTex2D::getTexCount() {
	return overall_tex_count_;
}

mutex gfx::GpuTex2D::overall_tex_list_mutex_;
vector<gfx::GpuTex2D*> gfx::GpuTex2D::overall_tex_list_;

vector<gfx::GpuTex2D*> gfx::GpuTex2D::getTexList() {
	return overall_tex_list_;
}

gfx::GpuTex2D::GpuTex2D(GarbageCollector *garbage_collector,
                        GLuint gl_internal_format, GLuint gl_format, 
                        GLuint gl_type, int width, int height, 
                        bool cuda_normalized_tex_coords, void *data, 
                        GLint filter_type) 
		: garbage_collector_(garbage_collector),
		  gl_format_(gl_format),
		  gl_internal_format_(gl_internal_format),
		  gl_type_(gl_type),
		  height_(height),
		  width_(width) {

	overall_tex_list_mutex_.lock();
	overall_tex_count_++;
	overall_tex_list_.push_back(this);
	overall_tex_list_mutex_.unlock();

	gfx::GLUtils::checkForOpenGLError("Error before generating glTexture");
	glGenTextures(1, &gl_name_);
	gfx::GLUtils::checkForOpenGLError("Error while generating glTexture");
	glBindTexture(GL_TEXTURE_2D, gl_name_);
	glTexImage2D(GL_TEXTURE_2D, 0, gl_internal_format_, width_, height_, 0, 
	             gl_format_, gl_type_, data);
	gfx::GLUtils::checkForOpenGLError("Error while creating glTexture");

	cudaChannelFormatKind type;
	int bit_depth = 0;
	switch(gl_type_) {
		case GL_UNSIGNED_BYTE:
			type        = cudaChannelFormatKindUnsigned;
			bit_depth   = 8;
			byte_count_ = 1;
			break;
		case GL_BYTE:
			type        = cudaChannelFormatKindSigned;
			bit_depth   = 8;
			byte_count_ = 1;
			break;
		case GL_UNSIGNED_SHORT:
			type        = cudaChannelFormatKindUnsigned;
			bit_depth   = 16;
			byte_count_ = 2;
			break;
		case GL_SHORT:
			type        = cudaChannelFormatKindSigned;
			bit_depth   = 16;
			byte_count_ = 2;
			break;
		case GL_UNSIGNED_INT:
			type        = cudaChannelFormatKindUnsigned;
			bit_depth   = 32;
			byte_count_ = 4;
			break;
		case GL_INT:
			type        = cudaChannelFormatKindSigned;
			bit_depth   = 32;
			byte_count_ = 4;
			break;
		case GL_FLOAT:
			type = cudaChannelFormatKindFloat;
			// TODO: distinguish between float16 and float32
			if(gl_internal_format_ == GL_RGBA16F||
			   gl_internal_format_ == GL_RGB16F ||
			   gl_internal_format_ == GL_RG16F  ||
			   gl_internal_format_ == GL_R16F) {
				bit_depth   = 16;
				byte_count_ = 2;
			} else {
				bit_depth   = 32;
				byte_count_ = 4;
			}
			break;
		default:
			break;
	}

	switch(gl_format_) {
		case GL_RED:
			channel_count_ = 1;
			break;
		case GL_RG:
			channel_count_ = 2;
			break;
		case GL_RGB:
			channel_count_ = 3;
			break;
		case GL_RGBA:
			channel_count_ = 4;
			break;
			//integer input
		case GL_RED_INTEGER:
			channel_count_ = 1;
			break;
		case GL_RG_INTEGER:
			channel_count_ = 2;
			break;
		case GL_RGB_INTEGER:
			channel_count_ = 3;
			break;
		case GL_RGBA_INTEGER:
			channel_count_ = 4;
			break;
		default:
			cout << "Invalid channel count!" << endl;
			assert(0);
			break;
	}

	// Register resource
	cudaGraphicsGLRegisterImage(&cuda_texture_resource_, gl_name_, GL_TEXTURE_2D,
	                            cudaGraphicsRegisterFlagsNone);
	gpuErrchk(cudaPeekAtLastError());

	// Map resource
	cudaGraphicsMapResources(1, &cuda_texture_resource_, 0);
	gpuErrchk(cudaPeekAtLastError());

	// Get array from this mapped resource
	cudaGraphicsSubResourceGetMappedArray(&cuda_array_reference_, 
	                                      cuda_texture_resource_, 0, 0);
	gpuErrchk(cudaPeekAtLastError());

	// Create this cuda texture:
	cudaResourceDesc res_desc;
	memset(&res_desc, 0, sizeof(res_desc));
	res_desc.resType = cudaResourceTypeArray;
	res_desc.res.array.array = cuda_array_reference_;//this is the only one that is allowed for binding it to a texture or surface

	cudaTextureDesc tex_desc;
	memset(&tex_desc, 0, sizeof(tex_desc));
	tex_desc.addressMode[0]   = cudaAddressModeBorder;
	tex_desc.addressMode[1]   = cudaAddressModeBorder;
	tex_desc.filterMode       = cudaFilterModeLinear;//cudaFilterModePoint //TODO: test filtering for unsigned char
	tex_desc.normalizedCoords = cuda_normalized_tex_coords;
	tex_desc.readMode         = cudaReadModeNormalizedFloat;//read the texture normalized.TODO: maybe as parameter
	if(type == cudaChannelFormatKindFloat) {
		tex_desc.filterMode = cudaFilterModeLinear;
		tex_desc.readMode   = cudaReadModeElementType;//somehow this tends to crash when variable is a float
	} else {
		tex_desc.filterMode = cudaFilterModePoint;
		if(bit_depth == 32) {
			tex_desc.readMode = cudaReadModeElementType;
		}
	}

	// Create texture object
	cudaCreateTextureObject(&cuda_texture_reference_, &res_desc, &tex_desc, NULL);
	gpuErrchk(cudaPeekAtLastError());

	//create surface object
	///TODO: check if this really works
	cudaCreateSurfaceObject(&cuda_surface_reference_, &res_desc);
	gpuErrchk(cudaPeekAtLastError());

	//this is the time where we change the texture settings(if we do this before binding the texture to cuda this binding fails.
	//and if we dont set these settings we for some reasons can't have bindless)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter_type);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter_type);

	//we want to show the problem to solve it //TODO!!!!!
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

	gl_handle_ = glGetTextureHandleARB(gl_name_);

	gfx::GLUtils::checkForOpenGLError("Error while obtaining the GL texture handle");
}

gfx::GpuTex2D::~GpuTex2D() {
	gfx::GLUtils::checkForOpenGLError("before destroying the gpu tex");
	overall_tex_count_--;

	//i think we should make sure that all the textures we want to use are mapped
	//https://github.com/ugovaretto/cuda-opengl-interop/blob/master/texture3d-write/simpleGL3DSurfaceWrite.cpp

	uint64_t handle = getGlHandle();
	thread::id this_id = this_thread::get_id();
	token_mutex_.lock();
	for(auto token : resident_token_) {
		thread::id id = token.first;
		if(id != this_id) {
			shared_ptr<bool> sharedToken = token.second;
			function<void()> cleanupTask = [sharedToken,handle]() {
				if(!(*sharedToken)) {//when the token is set to true then the handle has been cleared already
					if(glIsTextureHandleResidentARB(handle)) {
						glMakeTextureHandleNonResidentARB(handle);
					} else {
						assert(0);//the token should already ensure that the object exists.
					}

				}
				assert(!gfx::GLUtils::checkForOpenGLError("cleaning up the gpu handle"));
			};
			garbage_collector_->addToClean(id, cleanupTask);
		}
	}
	resident_token_.clear();//clear all token now
	// (this prevents the last resort cleanup method from making the handle non-resident)
	token_mutex_.unlock();

	gpuErrchk(cudaPeekAtLastError());
	cudaDestroyTextureObject(cuda_texture_reference_);
	gpuErrchk(cudaPeekAtLastError());
	cudaDestroySurfaceObject(cuda_surface_reference_);
	gpuErrchk(cudaPeekAtLastError());
	cudaGraphicsUnmapResources(1, &cuda_texture_resource_, 0);
	cudaGraphicsUnregisterResource(cuda_texture_resource_);//this throws an unknown error
	gpuErrchk(cudaPeekAtLastError());
	glDeleteTextures(1, &gl_name_);
	gfx::GLUtils::checkForOpenGLError("destroying the gpuTex");

	// TODO: check if the cudaDestroy functions are really supposed to work this way

	//i hope this forces the driver to free the memory
	glFinish();

	overall_tex_list_mutex_.lock();
	for(size_t i = overall_tex_list_.size() - 1; i >= 0; i--) {
		if(overall_tex_list_[i] == this) {
			overall_tex_list_[i] = overall_tex_list_[overall_tex_list_.size() - 1];
			overall_tex_list_.pop_back();
			overall_tex_list_mutex_.unlock();
			return;
		}
	}
	overall_tex_list_mutex_.unlock();
}

void gfx::GpuTex2D::uploadData(void *data) {
	cudaMemcpyToArray(cuda_array_reference_, 0, 0, data, 
	                  width_ * height_ * byte_count_ * channel_count_,
	                  cudaMemcpyHostToDevice);
}

void gfx::GpuTex2D::uploadData(void *data, int width, int height) {
	uploadData(data, 0, 0, width, height);
}

void gfx::GpuTex2D::uploadData(void *data, int x, int y, int width, int height) {
	cudaMemcpy2DToArray(cuda_array_reference_, x * byte_count_ * channel_count_,
	                    y, data, width * byte_count_ * channel_count_,
	                    width * byte_count_ * channel_count_, height,
	                    cudaMemcpyHostToDevice);
}

void gfx::GpuTex2D::downloadData(void *data) {
	downloadData(data, 0, 0, width_, height_);
}

void gfx::GpuTex2D::downloadData(void *data, int x, int y, int width, 
                                 int height) {
	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	cout << "indeed it is not functional!!!!!!! shit!!!!!! << i think the dpitch is not right" << endl;
	#endif
	cudaMemcpy2DFromArray(data, byte_count_ * channel_count_ * width,//dpitch (not sure about that) //step
	                      cuda_array_reference_, x * byte_count_ * channel_count_, 
	                      y,//x in bytes?
	                      width * byte_count_ * channel_count_, height,
	                      cudaMemcpyDeviceToHost);
	gpuErrchk(cudaPeekAtLastError());
}

void gfx::GpuTex2D::makeResidentInThisThread() {
	thread::id id = this_thread::get_id();
	uint64_t handle = getGlHandle();
	if(!glIsTextureHandleResidentARB(handle)) {
		glMakeTextureHandleResidentARB(handle);
		shared_ptr<bool> token = make_shared<bool>(false);//create the token!!!
		token_mutex_.lock();
		resident_token_[id] = token;
		token_mutex_.unlock();
		weak_ptr<bool> weak_token = token;
		function<void()> lastResortDelete = [weak_token, handle]() {
			if(!weak_token.expired()) {
				*weak_token.lock() = true;
				//make the handle non resident only if none of the token still exists for this thread
				glMakeTextureHandleNonResidentARB(handle);
				assert(!gfx::GLUtils::checkForOpenGLError("making a handle non resident in thread"));
			}
		};
		garbage_collector_->addToForceCollect(lastResortDelete);
	}
	assert(!gfx::GLUtils::checkForOpenGLError("Error at making the texture resident"));
}
