#ifndef FILE_THREAD_SAFE_FBO_VAO_H
#define FILE_THREAD_SAFE_FBO_VAO_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <mutex>
#include <thread>
#include <map>
#include <unordered_map>
#include <stack>
#include <vector>
#include <set>
#include <unordered_set>
#include <assert.h>

#include <iostream>



//typedef ThreadSafeGlContainer<&(ThreadSafeGlContainer::createFBO)> ThreadSafeFBO;//, &(ThreadSafeGlContainer::deleteFBO)

template <void (*genFun)(GLsizei, GLuint*), void (*delFun)(GLsizei, GLuint*)>
class ThreadSafeGl;


template<void (*genFun)(GLsizei,GLuint*), void (*delFun)(GLsizei,GLuint*)>
class ThreadSafeGlStorage{
    friend ThreadSafeGl<genFun,delFun>;
private:
    std::mutex mapMutex;
    //TODO: rename this from fbo to something else
    std::unordered_map<std::thread::id,std::unordered_map<GLuint*,std::weak_ptr<GLuint>>> fboDeleteMap;
    std::unordered_map<std::thread::id,std::unordered_set<std::shared_ptr<GLuint>>> fboMap;
    int debugFboCount=0;
    void addToList(std::thread::id id,std::shared_ptr<GLuint> obj){
        mapMutex.lock();
        fboMap[id].insert(obj);
        mapMutex.unlock();
    }
public:


    ThreadSafeGl<genFun,delFun>* createGlObject();

    void addToCleanupList(const std::unordered_map<std::thread::id,std::weak_ptr<GLuint >> &toAdd){
        mapMutex.lock();
        for(auto element: toAdd){
            fboDeleteMap[element.first][element.second.lock().get()] = element.second;
        }
        mapMutex.unlock();
    }


    // cleans up all the unused FBOs in this thread
    void garbageCollect(){
        std::thread::id id = std::this_thread::get_id();
        std::vector<GLuint> vec;
        mapMutex.lock();
        if(fboDeleteMap.count(id)){
            for(auto obj : fboDeleteMap[id]){
                std::shared_ptr<GLuint> ob = obj.second.lock();
                if(ob == nullptr){
                    assert(0);
                }
                if(ob != nullptr){
                    //remove the object from the list and assemble the list for the deallocation process
                    fboMap[id].erase(ob);
                    vec.push_back(*ob);
                }
            }
            //remove the list of objects we want to erase because we will erase them after the mutex
            fboDeleteMap.erase(id);
        }
        mapMutex.unlock();
        if(vec.size()!=0){
            delFun(vec.size(),&vec[0]);
            debugFboCount -= vec.size();
        }

    }

    //TODO: implement this!!
    void forceGarbageCollect(){
        std::thread::id id = std::this_thread::get_id();
        std::vector<GLuint> vec;
        mapMutex.lock();
        fboDeleteMap.erase(id);
        if(fboMap.count(id)){
            for(std::shared_ptr<GLuint> ob: fboMap[id]){
                vec.push_back(*ob);
            }
        }
        fboMap.erase(id);
        mapMutex.unlock();
        if(vec.size()!=0){
            delFun(vec.size(),&vec[0]);
            debugFboCount -= vec.size();
        }


    }

    void debugIncreaseCount(int c=1){
        debugFboCount += c;
    }

    int getTotalFboCount(){
        int count=0;
        mapMutex.lock();
        for(auto element: fboDeleteMap){
            count += element.second.size();
        }
        mapMutex.unlock();
        return debugFboCount;
    }



};


template<void (*genFun)(GLsizei,GLuint*), void (*delFun)(GLsizei,GLuint*)>
class ThreadSafeGl{
private:
    std::mutex mapMutex;
    std::unordered_map<std::thread::id,std::weak_ptr<GLuint>> threadMap;
    ThreadSafeGlStorage<genFun,delFun>* threadStorage;
public:
    bool existsInThisThread(){
        std::thread::id id = std::this_thread::get_id();
        mapMutex.lock();
        bool result = threadMap.count(id) != 0;
        if(result){
            result = !threadMap[id].expired();
        }
        mapMutex.unlock();
        return result;
    }

    GLuint get(){
        std::thread::id id = std::this_thread::get_id();
        std::shared_ptr<GLuint> bo = nullptr;//buffer object
        mapMutex.lock();
        if(threadMap.count(id) == 0){
            bo = std::make_shared<GLuint>();
            genFun(1,bo.get());
            //glGenFramebuffers(1,&fbo);
            //std::cout << "debug fbo id " << bo << std::endl;
            threadMap[id]=bo;
            threadStorage->addToList(id,bo);
            threadStorage->debugIncreaseCount();
        }else{
            bo=threadMap.at(id).lock();
            if(bo == nullptr){
                // the element has been deleted due to ending the thread and calling force garbage collect.
                assert(0); // this should only happen at the end of a thread.... so why is this happening then?
                bo = std::make_shared<GLuint>();
                genFun(1,bo.get());
                threadMap[id]=bo;
                threadStorage->addToList(id,bo);
                threadStorage->debugIncreaseCount();
            }
        }
        mapMutex.unlock();
        return *bo;
    }

    ThreadSafeGl(ThreadSafeGlStorage<genFun,delFun>* storage){
        threadStorage=storage;
    }

    ~ThreadSafeGl(){
        mapMutex.lock();
        threadStorage->addToCleanupList(threadMap);
        mapMutex.unlock();
    }
};

template<void (*genFun)(GLsizei,GLuint*), void (*delFun)(GLsizei,GLuint*)>
inline ThreadSafeGl<genFun, delFun> *ThreadSafeGlStorage<genFun,delFun>::createGlObject(){
    return new ThreadSafeGl<genFun,delFun>(this);
}



//create the fbo classes from the templated manager
inline void fboGenFun(GLsizei count,GLuint *fbos){
    glGenFramebuffers(count,fbos);

}

inline void fboDelFun(GLsizei count, GLuint *fbos){
    glDeleteFramebuffers(count,fbos);
}


typedef ThreadSafeGl<fboGenFun,fboDelFun> ThreadSafeFBO;
typedef ThreadSafeGlStorage<fboGenFun,fboDelFun> ThreadSafeFBOStorage;


//do the same for the VAOs
inline void vaoGenFun(GLsizei count,GLuint *vaos){
    glGenVertexArrays(count,vaos);
}

inline void vaoDelFun(GLsizei count, GLuint *vaos){
    glDeleteVertexArrays(count,vaos);
}

//using ThreadSafeFBO =  ThreadSafeGl<fboGenFun,fboGenFun>; //thats another way of achieving the templating
typedef ThreadSafeGl<vaoGenFun,vaoDelFun> ThreadSafeVAO;
typedef ThreadSafeGlStorage<vaoGenFun,vaoDelFun> ThreadSafeVAOStorage;




/*
class basicFBOGeneratorDestructor{
public:
    static void del(GLuint count,GLuint *fbos){
        glDeleteFramebuffers(count,fbos);
    }
    static void gen(GLuint count,GLuint *fbos){
        glGenFramebuffers(count,fbos);
    }

};

typedef ThreadSafeGl<basicFBOGeneratorDestructor> ThreadSafeFBO;
typedef ThreadSafeGlStorage<basicFBOGeneratorDestructor> ThreadSafeFBOStorage;


template class ThreadSafeGl<basicFBOGeneratorDestructor>;

template class ThreadSafeGlStorage<basicFBOGeneratorDestructor>;
*/

//now do the templating work


/*
something thread local
 // Example program
#include <iostream>
#include <string>
#include <thread>
using namespace std;
class Test{
    public:
    string m_name;
    Test(string name){
        m_name=name;
        cout  << "constructor" << endl;
    }
    ~Test(){
        cout << "destructor" << endl;
    }
    void something(){
        cout << "something from " << m_name << endl;
    }

};


thread_local Test *test;


void proc(){
    test=new Test("thread");
    test->something();
    delete test;

}
int main()
{
    test = new Test("main");

    thread ttest(proc);
    ttest.join();


    test->something();
    delete test;



}
*/



#endif
