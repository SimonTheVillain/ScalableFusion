#include "gpu_buffer.h"

#include <iostream>
#include <assert.h>

#include <cuda/gpu_errchk.h>

using namespace std;

template<typename T>
GpuBuffer<T>::GpuBuffer(size_t element_count, GLuint buffer_type, 
                        bool debug_no_gpu) {
	debug_absolute_size_ = element_count;
	debug_ = debug_no_gpu;

	if(!debug_no_gpu) {
		//create opengl buffer and connect it to cuda:
		glGenBuffers(1, &gl_buffer_);
		glBindBuffer(buffer_type, gl_buffer_);
		glBufferData(buffer_type, sizeof(T) * element_count, 0, GL_DYNAMIC_DRAW);

		cudaError_t error_test;
		error_test = cudaGraphicsGLRegisterBuffer(&cuda_graphics_resource_, 
		                                          gl_buffer_,
		                                          cudaGraphicsMapFlagsNone);//TODO: test these
		gpuErrchk(error_test);
		error_test = cudaGraphicsMapResources(1, &cuda_graphics_resource_);//stream 0
		gpuErrchk(error_test);
		size_t size;
		error_test = cudaGraphicsResourceGetMappedPointer(
				(void**) &cuda_resource_ptr_,
				&size,cuda_graphics_resource_);
		gpuErrchk(error_test);
		error_test = cudaGraphicsUnmapResources(1, &cuda_graphics_resource_);//stream0
		gpuErrchk(error_test);

		error_test = cudaGraphicsMapResources(1, &cuda_graphics_resource_);//stream 0
		gpuErrchk(error_test);
	}

	//fill the structures with one giant free block
	free_slots_[element_count].insert(0);
	starts_[0] = element_count;
	ends_[element_count - 1] = element_count;
}

template<typename T>
GpuBuffer<T>::~GpuBuffer() {
	cudaError_t error_test;

	error_test = cudaGraphicsUnmapResources(1, &cuda_graphics_resource_);
	gpuErrchk(error_test);
	error_test = cudaGraphicsUnregisterResource(cuda_graphics_resource_);
	gpuErrchk(error_test);
	glDeleteBuffers(1, &gl_buffer_);
}

template<typename T>
shared_ptr<GpuBufferConnector<T>> GpuBuffer<T>::getBlock(size_t element_count) {

	if(element_count == 0) {
		cout << "hey maybe this is killing everything" << endl;
		shared_ptr<GpuBufferConnector<T>> buf(new GpuBufferConnector<T>(this, 0, 0));
		return buf;
		assert(0);
	}
	mutex_.lock();

	//first check if there are blocks of the requested size
	map<size_t, set<uint32_t>>::iterator it =
			free_slots_.lower_bound(element_count);

	if(it == free_slots_.end()) {
		shared_ptr<GpuBufferConnector<T>> empty;
		mutex_.unlock();
		cout << "Error: Ran out of GPU memory for geometry" << endl;
		assert(0);//No free elements. everything from here on will be pain
		return empty;
	}

	//list of free blocks of this size
	set<uint32_t> &list_free_of_size = it->second;
	//size of these blocks
	uint32_t size_of_free = it->first;

	//pointer to the first and last element of the chosen block
	uint32_t first = *list_free_of_size.begin();
	uint32_t last  = first + size_of_free - 1;

	//index, the adress of the block within the memory
	uint32_t index = last - element_count + 1;

	//delete the ending points from the lists:
	if(starts_.find(first) == starts_.end()) {
		cout << "this should not happen, how did it happen then?" << endl;
		assert(0);
	}
	starts_.erase(starts_.find(first));
	ends_.erase(ends_.find(last));
	list_free_of_size.erase(list_free_of_size.begin());
	if(list_free_of_size.empty()) {
		//if the list of free slots of this size is empty we also delete
		//this element in the higher level map
		free_slots_.erase(it);
	}

	if(index != first){// if the block is not exactly the requested size
		//but also create new entries for the object we split apart
		// (if necessary) and we are not taking a full block
		uint32_t new_size = size_of_free - element_count;
		starts_[first] = new_size;
		ends_[first + new_size - 1] = new_size;
		free_slots_[new_size].insert(first);
	}

	shared_ptr<GpuBufferConnector<T>> buffer(new GpuBufferConnector<T>(
			this, index, element_count));
	mutex_.unlock();
	return buffer;
}

// TODO: HEY! THIS SHOUTS FOR RECURSION!!!!!!
template<typename T>
void GpuBuffer<T>::rejoinBlock(GpuBufferConnector<T> *block, bool first_level) {
	if(block->getSize() == 0) {
		cout << "why does this happen? this really should never ever happen" << endl;
		return;
		assert(0);
	}
	if(first_level) {
		mutex_.lock();
	}
	uint32_t size = block->size_;
	uint32_t first = block->index_;
	uint32_t last = first + size - 1;
	//check if there is a starting or endpoint overlapping with this
	//in case stitch it together
	map<uint32_t, uint32_t>::iterator it_start = starts_.find(last + 1);
	map<uint32_t, uint32_t>::iterator it_end = ends_.find(first - 1);

	if(it_start != starts_.end()) {
		//there is a free block directly after the one we want to insert:
		//increase the size of our current block
		uint32_t other_size = it_start->second;
		uint32_t other_start = last + 1;
		block->size_ += other_size;

		// now erase everything that leads up to this
		map<size_t, set<uint32_t>>::iterator it = free_slots_.find(other_size);
		if(it == free_slots_.end()) {
			assert(0);//this should never happen
			// if the structure were consistent there
			//has to be something found in here
		}

		set<uint32_t> &list_free_of_size = it->second;

		if(list_free_of_size.find(other_start) == list_free_of_size.end()) {
			assert(0);
		}
		list_free_of_size.erase(list_free_of_size.find(other_start));
		if(list_free_of_size.empty()) {
			free_slots_.erase(it);
		}

		starts_.erase(it_start);
		ends_.erase(ends_.find(other_start + other_size - 1));

		//recursively call this function to reintroduce the block
		rejoinBlock(block, false);

		return;
	}

	if(it_end != ends_.end()) {
		//there is a free block directly before the one we want to insert:
		//increase the size of our current block+
		//adapt the index
		uint32_t other_size = it_end->second;
		uint32_t other_start = first - other_size;
		block->index_ -= other_size;
		block->size_ += other_size;

		// now erase everything that leads up to this
		map<size_t, set<uint32_t>>::iterator it = free_slots_.find(other_size);
		if(it == free_slots_.end()) {
			assert(0);//this should never happen
			// if the structure were consistent there
			//has to be something found in here
		}

		set<uint32_t> &list_free_of_size = it->second;
		if(list_free_of_size.find(other_start) == list_free_of_size.end()) {
			assert(0);//this should never happen either
		}
		list_free_of_size.erase(list_free_of_size.find(other_start));
		if(list_free_of_size.empty()) {
			free_slots_.erase(it);
		}

		starts_.erase(starts_.find(other_start));
		ends_.erase(it_end);

		//recursively call this function to reintroduce the block
		rejoinBlock(block, false);

		return;
	}

	//since we found everything we
	starts_[first] = size;
	ends_[last] = size;
	free_slots_[size].insert(first);

	mutex_.unlock();
}

template<typename T>
void GpuBuffer<T>::upload(T *data, uint32_t index, uint32_t count) {
	cudaError_t error_test;
	error_test = cudaMemcpy(&(cuda_resource_ptr_[index]), data, sizeof(T) * count,
	                        cudaMemcpyHostToDevice);
	gpuErrchk(error_test);
}

template<typename T>
void GpuBuffer<T>::download(T *data, uint32_t index, uint32_t count) {
	cudaError_t error_test;
	error_test = cudaMemcpy(data, &(cuda_resource_ptr_[index]), sizeof(T) * count,
	                        cudaMemcpyDeviceToHost);
	 cudaDeviceSynchronize();
	 gpuErrchk(error_test);
}

template<typename T>
uint32_t GpuBuffer<T>::getUsedElements() {
	mutex_.lock();
	int count = 0;
	for (auto size : starts_) {
		count += size.second;
	}
	mutex_.unlock();
	return debug_absolute_size_ - count;
}

template<typename T>
uint32_t GpuBuffer<T>::getFreeElements() {
	mutex_.lock();
	int count = 0;
	for (auto size : starts_) {
		count += size.second;
	}
	mutex_.unlock();
	return count;
}

template<typename T>
GpuBufferConnector<T>::GpuBufferConnector(GpuBuffer<T> *buffer, uint32_t index,
                                          uint32_t size)
		: buffer_(buffer),
		  index_(index),
		  size_(size) {
}

template<typename T>
GpuBufferConnector<T>::~GpuBufferConnector() {
	buffer_->rejoinBlock(this);
}

template<typename T>
void GpuBufferConnector<T>::upload(T *data, uint32_t offset, uint32_t count) {
	if(count == 0) {
		return;
	}
	buffer_->upload(data, index_ + offset, count);
}

template<typename T>
void GpuBufferConnector<T>::download(T *data, uint32_t offset, uint32_t count) {
	if(count == 0){
		return;
	}
	buffer_->download(data, index_ + offset, count);
}

template class GpuBufferConnector<GpuVertex>;
template class GpuBufferConnector<Eigen::Vector2f>;
template class GpuBufferConnector<GpuTriangle>;
template class GpuBufferConnector<GpuPatchInfo>;
template class GpuBufferConnector<GLint>;


template class GpuBuffer<GpuVertex>;
template class GpuBuffer<Eigen::Vector2f>;
template class GpuBuffer<GpuTriangle>;
template class GpuBuffer<GpuPatchInfo>;
template class GpuBuffer<GLint>;
