#ifndef FILE_THREAD_SAFE_FBO_VAO_H
#define FILE_THREAD_SAFE_FBO_VAO_H

#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <assert.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace std;

template <void (*genFun) (GLsizei, GLuint*), void (*delFun) (GLsizei, GLuint*)>
class ThreadSafeGl;

template<void (*genFun) (GLsizei, GLuint*), void (*delFun) (GLsizei, GLuint*)>
class ThreadSafeGlStorage {
	friend ThreadSafeGl<genFun, delFun>;

public:

	ThreadSafeGl<genFun, delFun>* createGlObject();

	void addToCleanupList(const unordered_map<thread::id, 
	                      weak_ptr<GLuint >> &to_add) {
		map_mutex_.lock();
		for(auto element: to_add) {
			fbo_delete_map_[element.first][element.second.lock().get()] = element.second;
		}
		map_mutex_.unlock();
	}

	// cleans up all the unused FBOs in this thread
	void garbageCollect() {
		thread::id id = this_thread::get_id();
		vector<GLuint> vec;
		map_mutex_.lock();
		if(fbo_delete_map_.count(id)) {
			for(auto obj : fbo_delete_map_[id]) {
				shared_ptr<GLuint> ob = obj.second.lock();
				if(ob == nullptr) {
					assert(0);
				}
				if(ob != nullptr) {
					//remove the object from the list and assemble the list for the deallocation process
					fbo_map_[id].erase(ob);
					vec.push_back(*ob);
				}
			}
			//remove the list of objects we want to erase because we will erase them after the mutex
			fbo_delete_map_.erase(id);
		}
		map_mutex_.unlock();
		if(vec.size() != 0) {
			delFun(vec.size(), &vec[0]);
			debug_fbo_count_ -= vec.size();
		}
	}

	//TODO: implement this!!
	void forceGarbageCollect() {
		thread::id id = this_thread::get_id();
		vector<GLuint> vec;
		map_mutex_.lock();
		fbo_delete_map_.erase(id);
		if(fbo_map_.count(id)) {
			for(shared_ptr<GLuint> ob : fbo_map_[id]) {
				vec.push_back(*ob);
			}
		}
		fbo_map_.erase(id);
		map_mutex_.unlock();
		if(vec.size() != 0) {
			delFun(vec.size(), &vec[0]);
			debug_fbo_count_ -= vec.size();
		}
	}

	void debugIncreaseCount(int c = 1) {
		debug_fbo_count_ += c;
	}

	int getTotalFboCount() {
		int count = 0;
		map_mutex_.lock();
		for(auto element : fbo_delete_map_) {
			count += element.second.size();
		}
		map_mutex_.unlock();
		return debug_fbo_count_;
	}

private:
	
	void addToList_(thread::id id, shared_ptr<GLuint> obj) {
		map_mutex_.lock();
		fbo_map_[id].insert(obj);
		map_mutex_.unlock();
	}

	mutex map_mutex_;
	//TODO: rename this from fbo to something else
	unordered_map<thread::id, unordered_map<GLuint*, weak_ptr<GLuint>>> fbo_delete_map_;
	unordered_map<thread::id, unordered_set<shared_ptr<GLuint>>> fbo_map_;
	int debug_fbo_count_ = 0;

};


template<void (*genFun) (GLsizei, GLuint*), void (*delFun) (GLsizei, GLuint*)>
class ThreadSafeGl {
public:

	ThreadSafeGl(ThreadSafeGlStorage<genFun, delFun> *storage) 
			: thread_storage_(storage) {
	}

	~ThreadSafeGl() {
		map_mutex_.lock();
		thread_storage_->addToCleanupList(thread_map_);
		map_mutex_.unlock();
	}

	bool existsInThisThread() {
		thread::id id = this_thread::get_id();
		map_mutex_.lock();
		bool result = thread_map_.count(id) != 0;
		if(result) {
			result = !thread_map_[id].expired();
		}
		map_mutex_.unlock();
		return result;
	}

	GLuint get() {
		thread::id id = this_thread::get_id();
		shared_ptr<GLuint> bo = nullptr;//buffer object
		map_mutex_.lock();
		if(thread_map_.count(id) == 0) {
			bo = make_shared<GLuint>();
			genFun(1, bo.get());
			thread_map_[id] = bo;
			thread_storage_->addToList_(id, bo);
			thread_storage_->debugIncreaseCount();
		} else {
			bo = thread_map_.at(id).lock();
			if(bo == nullptr) {
				// the element has been deleted due to ending the thread and calling force garbage collect.
				assert(0); // this should only happen at the end of a thread.... so why is this happening then?
				bo = make_shared<GLuint>();
				genFun(1, bo.get());
				thread_map_[id] = bo;
				thread_storage_->addToList_(id, bo);
				thread_storage_->debugIncreaseCount();
			}
		}
		map_mutex_.unlock();
		return *bo;
	}

private:
	mutex map_mutex_;
	unordered_map<thread::id,weak_ptr<GLuint>> thread_map_;
	ThreadSafeGlStorage<genFun,delFun>* thread_storage_;
	
};

template<void (*genFun) (GLsizei, GLuint*), void (*delFun) (GLsizei, GLuint*)>
inline ThreadSafeGl<genFun, delFun> *ThreadSafeGlStorage<genFun, delFun>::createGlObject() {
	return new ThreadSafeGl<genFun, delFun>(this);
}

//create the fbo classes from the templated manager
inline void fboGenFun(GLsizei count, GLuint *fbos) {
	glGenFramebuffers(count, fbos);

}

inline void fboDelFun(GLsizei count, GLuint *fbos) {
	glDeleteFramebuffers(count, fbos);
}

typedef ThreadSafeGl<fboGenFun, fboDelFun> ThreadSafeFBO;
typedef ThreadSafeGlStorage<fboGenFun, fboDelFun> ThreadSafeFBOStorage;

//do the same for the VAOs
inline void vaoGenFun(GLsizei count, GLuint *vaos) {
	glGenVertexArrays(count, vaos);
}

inline void vaoDelFun(GLsizei count, GLuint *vaos) {
	glDeleteVertexArrays(count, vaos);
}

typedef ThreadSafeGl<vaoGenFun, vaoDelFun> ThreadSafeVAO;
typedef ThreadSafeGlStorage<vaoGenFun, vaoDelFun> ThreadSafeVAOStorage;

#endif // FILE_THREAD_SAFE_FBO_VAO_H
