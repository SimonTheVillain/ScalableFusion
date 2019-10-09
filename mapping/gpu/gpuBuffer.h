#ifndef FILE_GPU_BUFFER_H
#define FILE_GPU_BUFFER_H

#include <mutex>
#include <set>
#include <map>
#include <memory>

#include <GL/glew.h>
#include <cuda.h>
#include <cuda_gl_interop.h>

#include "../cuda/gpuMeshStructure.h"

using namespace std;

template<typename T>
class GpuBuffer;

//TODO: rename BufferConnector to bufferHandle / slot? (except for when it really is more a connector than a slot
template<typename T>
class GpuBufferConnector {
	friend GpuBuffer<T>;

public:

	~GpuBufferConnector();

	uint32_t getStartingIndex() {
		return index_;
	}

	T* getStartingPtr() {
		return &(buffer_->getCudaPtr()[index_]);
	}

	void upload(T *data, uint32_t offset, uint32_t count);

	void upload(T *data) {
		upload(data, 0, size_);
	}

	void download(T *data, uint32_t offset, uint32_t count);

	void download(T *data) {
		download(data, 0, size_);
	}

	uint32_t getSize() {
		return size_;
	}

private:

	GpuBufferConnector(GpuBuffer<T> *buffer, uint32_t index, uint32_t size);

	uint32_t index_;//the starting index for this slot
	uint32_t size_;

	GpuBuffer<T> *buffer_ = 0;

};

//TODO: rename this to GpuBufferCollection
template<typename T>
class GpuBuffer {
public:

	GpuBuffer(size_t element_count, GLuint buffer_type = GL_ARRAY_BUFFER,
	          bool debug_no_gpu = false);

	~GpuBuffer();

	shared_ptr<GpuBufferConnector<T>> getBlock(size_t element_count);

	void rejoinBlock(GpuBufferConnector<T> *block, bool first_level = true);

	void upload(T *data, uint32_t index, uint32_t count);

	void download(T *data, uint32_t index, uint32_t count);

	GLuint getGlName() {
		return gl_buffer_;
	}

	T *getCudaPtr() {
		return cuda_resource_ptr_;
	}

	uint32_t getUsedElements();

	uint32_t getFreeElements();

private:

	mutex mutex_;

	//store the free slots of a certain size
	//first its only one, of full size, smaller ones get split off of this one
	//the free slot gets moved down the according size
	//problems:
	//1) giving back one element and connecting this whith existing slots
	//probably doable with start and end map!
	//2) finding the next best slot (next biggest key)
	// maps are sorted
	// https://stackoverflow.com/questions/1660195/c-how-to-find-the-biggest-key-in-a-stdmap

	//key: size, value: adress (maybe replace this vector with a set(sorted))
	map<size_t, set<uint32_t>> free_slots_;

	//key: starting index, value: size
	map<uint32_t, uint32_t> starts_;

	//key: ending index, value: size
	map<uint32_t, uint32_t> ends_;

	GLuint gl_buffer_;
	cudaGraphicsResource_t cuda_graphics_resource_;
	T *cuda_resource_ptr_;
	bool debug_ = false;
	size_t debug_absolute_size_;//debug because it is unused

};

typedef GpuBuffer<GpuVertex> VertexBuffer;
typedef GpuBuffer<Eigen::Vector2f> TexCoordBuffer;
typedef GpuBuffer<GpuTriangle> TriangleBuffer;
typedef GpuBuffer<GpuPatchInfo> PatchInfoBuffer;
typedef GpuBuffer<GLint> IndexToPatchInfoBuffer;

typedef GpuBufferConnector<GpuVertex> VertexBufConnector;
typedef GpuBufferConnector<Eigen::Vector2f> TexCoordBufConnector;
typedef GpuBufferConnector<GpuTriangle> TriangleBufConnector;//TODO: see if this checks out... otherwise go back to "typedef"
typedef GpuBufferConnector<GpuPatchInfo> PatchInfoBufConnector;

void testBuffer();

#endif // FILE_GPU_BUFFER_H
