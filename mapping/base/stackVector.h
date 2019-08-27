//
// Created by simon on 5/5/19.
//

#ifndef SUPERMAPPING_STACKVECTOR_H
#define SUPERMAPPING_STACKVECTOR_H

#include <vector>
#include <assert.h>

//idea similar to this: we want to have a container that lives on stack as long as possible
//https://chromium.googlesource.com/chromium/chromium/+/master/base/stack_container.h
//stackoverflow:
//https://stackoverflow.com/questions/354442/looking-for-c-stl-like-vector-class-but-using-stack-storage



template<typename T,std::size_t static_capacity>
class stack_vector{
public:
    T stack[static_capacity];
    std::size_t count = 0;
    T* heap = nullptr;
    std::size_t heap_capacity=0;

    void push_back(T in){
        if(count < static_capacity){
            //nothing if we are within stack
            stack[count] = in;
        }else{
            std::size_t delta = count-static_capacity;
            //if we are running out of heap we need to reallocate some more

            if(delta+1 > heap_capacity){
                T* oldHeap = heap;
                heap_capacity = heap_capacity*2;
                if(heap_capacity==0){
                    heap_capacity=2;
                }
                heap = new T[heap_capacity];
                for(std::size_t i=0;i<delta;i++){
                    heap[i] = oldHeap[i];
                }
                delete[] oldHeap;
            }
            heap[delta] = in;
        }
        count++;
    }
    void pop_back(){
        count--;

    }
    void emplace_back(){
        if(count < static_capacity){
            //nothing if we are within stack
        }else{
            std::size_t delta = count-static_capacity;
            //if we are running out of heap we need to reallocate some more
            if(delta+1 > heap_capacity){
                T* oldHeap = heap;
                heap_capacity = heap_capacity*2;
                if(heap_capacity==0){
                    heap_capacity=2;
                }
                heap = new T[heap_capacity];
                for(std::size_t i=0;i<delta;i++){
                    heap[i] = oldHeap[i];
                }
                delete[] oldHeap;
            }
        }
        count++;
    }
    std::size_t size(){
        return count;
    }

    T& operator[](std::size_t i){
        if(i<static_capacity){
            return stack[i];
        }
        return heap[i-static_capacity];
    }

    stack_vector(){
        //the constructor doesn't do much... since we don't really allocate anything
    }

    stack_vector(const stack_vector<T,static_capacity>  &in){ //<T,static_capacity> (why is this not called?
        heap_capacity = in.heap_capacity;
        count = in.count;
        memcpy(stack,in.stack,sizeof(T)*std::min(static_capacity, count));
        if(count > static_capacity){
            heap = new T[heap_capacity];

            size_t delta = count - static_capacity;
            memcpy(heap,in.heap,sizeof(T)*delta);

        }
    }


    ~stack_vector(){
        if(heap!=nullptr){
            delete[] heap;
        }
    }

    //copy assignment
    stack_vector& operator=(const stack_vector& that){
        heap_capacity = that.heap_capacity;
        count = that.count;
        memcpy(stack,that.stack,sizeof(T)*std::min(static_capacity, count));
        if(count > static_capacity){
            heap = new T[heap_capacity];

            size_t delta = count - static_capacity;
            memcpy(heap,that.heap,sizeof(T)*delta);

        }

        //std::cout << "calling copy assignment" << std::endl;
        return *this;
    }



    // move assignment
    stack_vector& operator=(stack_vector&& that){

        heap_capacity = that.heap_capacity;
        count = that.count;
        memcpy(stack,that.stack,sizeof(T)*std::min(static_capacity, count));
        heap = that.heap;
        that.count = 0;
        that.heap = nullptr;
        //std::cout << "calling move assignment" << std::endl;
        return *this;
    }


    //copy/move constructor (i don't think this applies in this case)
    /*
    stack_vector& operator=(T that) noexcept {

    }
     */

    T& back(){
        size_t i = count-1;
        if(i<static_capacity){
            return stack[i];
        }
        return heap[i-static_capacity];
    }

};


#endif //SUPERMAPPING_STACKVECTOR_H
