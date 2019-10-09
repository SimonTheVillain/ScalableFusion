#ifndef FILE_GPU_TEX_H
#define FILE_GPU_TEX_H

#include <string>
#include <vector>
#include <unordered_map>
#include <thread>
#include <mutex>

#include <GL/glew.h>
#include <cuda_gl_interop.h>

using namespace std;

class GarbageCollector;

// TODO: incorporate cuda streams into the whole mess!!!!
// TODO: create a few constants for the constructor to
//       easily initialize the most common texture types
namespace gfx {

class GpuTex2D {
public:

	// TODO: make the garbage collector mandatory
	GpuTex2D(GarbageCollector *garbage_collector, GLuint gl_internal_format,
	         GLuint gl_format, GLuint gl_type, int width, int height,
	         bool cuda_normalized_tex_coords, void* data = NULL,
	         GLint filter_type = GL_LINEAR);

	~GpuTex2D();

	cudaTextureObject_t      getCudaTextureObject() {return cuda_texture_reference_;}
	cudaSurfaceObject_t      getCudaSurfaceObject() {return cuda_surface_reference_;}
	GLuint                   getGlFormat() {return gl_format_;}
	uint64_t                 getGlHandle() {return gl_handle_;}
	GLuint                   getGlInternalFormat() {return gl_internal_format_;}
	GLuint                   getGlName() {return gl_name_;}
	GLuint                   getGlType() {return gl_type_;};
	static int               getTexCount();
	static vector<GpuTex2D*> getTexList();
	int                      getWidth() {return width_;}
	int                      getHeight() {return height_;}

	void uploadData(void *data); //if size == 0 we only load
	void uploadData(void *data, int width_, int height_);
	void uploadData(void *data, int x, int y, int width, int height);

	void downloadData(void *data);
	void downloadData(void *data, int x, int y, int width, int height);

	void makeResidentInThisThread();

	string name;

private:

	GLuint   gl_format_;
	uint64_t gl_handle_;
	GLuint   gl_internal_format_;
	GLuint   gl_name_;
	GLuint   gl_type_;

	struct cudaGraphicsResource* cuda_texture_resource_;
	cudaResourceDesc             cuda_res_desc_;
	cudaTextureObject_t          cuda_texture_reference_;
	cudaSurfaceObject_t          cuda_surface_reference_;
	cudaArray_t                  cuda_array_reference_;

	int channel_count_;
	int byte_count_;

	int height_;
	int width_;

	GarbageCollector *garbage_collector_;
	mutex             token_mutex_;
	unordered_map<thread::id, shared_ptr<bool>> resident_token_;

	static int               overall_tex_count_;
	static vector<GpuTex2D*> overall_tex_list_;
	static mutex             overall_tex_list_mutex_;

};

} // namespace gfx

#endif // FILE_GPU_TEX_H
