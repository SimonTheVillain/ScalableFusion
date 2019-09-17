#ifndef FILE_STACK_VECTOR_H
#define FILE_STACK_VECTOR_H

#include <vector>
#include <assert.h>

//idea similar to this: we want to have a container that lives on stack as long as possible
//https://chromium.googlesource.com/chromium/chromium/+/master/base/stack_container.h
//stackoverflow:
//https://stackoverflow.com/questions/354442/looking-for-c-stl-like-vector-class-but-using-stack-storage

using namespace std;

template<typename T, size_t StaticCapacity>
class StackVector {
public:

	StackVector() {
		//the constructor doesn't do much... since we don't really allocate anything
	}

	StackVector(const StackVector<T, StaticCapacity> &in) 
			: heap_capacity(in.heap_capacity),
			  count(in.count) { //<T,static_capacity> (why is this not called?
		memcpy(stack, in.stack, sizeof(T) * min(StaticCapacity, count));
		if(count > StaticCapacity) {
			heap = new T[heap_capacity];
			size_t delta = count - StaticCapacity;
			memcpy(heap, in.heap, sizeof(T) * delta);
		}
	}

	~StackVector() {
		if(heap != nullptr) {
			delete[] heap;
		}
	}

	void pushBack(T in) {
		if(count < StaticCapacity) {
			// Nothing if we are within stack
			stack[count] = in;
		} else {
			size_t delta = count - StaticCapacity;
			// If we are running out of heap we need to reallocate some more
			if(delta + 1 > heap_capacity) {
				T* old_heap = heap;
				heap_capacity = heap_capacity * 2;
				if(heap_capacity == 0) {
					heap_capacity = 2;
				}
				heap = new T[heap_capacity];
				for(size_t i = 0; i < delta; i++) {
					heap[i] = old_heap[i];
				}
				delete[] old_heap;
			}
			heap[delta] = in;
		}
		count++;
	}

	void popBack() {
		count--;
	}

	void emplaceBack() {
		if(count < StaticCapacity) {
			// Nothing if we are within stack
		} else {
			size_t delta = count - StaticCapacity;
			// If we are running out of heap we need to reallocate some more
			if(delta + 1 > heap_capacity) {
				T* old_heap = heap;
				heap_capacity = heap_capacity * 2;
				if(heap_capacity == 0) {
					heap_capacity = 2;
				}
				heap = new T[heap_capacity];
				for(size_t i = 0; i< delta; i++) {
					heap[i] = old_heap[i];
				}
				delete[] old_heap;
			}
		}
		count++;
	}

	T& back() {
		size_t i = count - 1;
		if(i < StaticCapacity) {
			return stack[i];
		} else {
			return heap[i-StaticCapacity];
		}
	}

	size_t size() {
		return count;
	}

	T& operator[](size_t i) {
		if(i < StaticCapacity) {
			return stack[i];
		} else {
			return heap[i - StaticCapacity];
		}
	}

	//copy assignment
	StackVector& operator=(const StackVector& that) {
		heap_capacity = that.heap_capacity;
		count = that.count;
		memcpy(stack, that.stack, sizeof(T) * min(StaticCapacity, count));
		if(count > StaticCapacity) {
			heap = new T[heap_capacity];
			size_t delta = count - StaticCapacity;
			memcpy(heap, that.heap, sizeof(T) * delta);
		}
		return *this;
	}

	// move assignment
	StackVector& operator=(StackVector&& that) {
		heap_capacity = that.heap_capacity;
		count = that.count;
		memcpy(stack, that.stack, sizeof(T) * min(StaticCapacity, count));
		heap = that.heap;
		that.count = 0;
		that.heap  = nullptr;
		return *this;
	}

	T  stack[StaticCapacity];
	T* heap = nullptr;
	size_t count = 0;
	size_t heap_capacity = 0;
};

#endif
