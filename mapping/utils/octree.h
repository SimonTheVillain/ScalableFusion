//https://geidav.wordpress.com/2014/08/18/advanced-octrees-2-node-representations/
//https://geidav.wordpress.com/2014/11/18/advanced-octrees-3-non-static-octrees/
#ifndef FILE_OCTREE_H
#define FILE_OCTREE_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <mutex>
#include <iostream>
//#include <tuple>

//more than these and we split them up
#define UPPER_LIMIT_OBJECTS 100
//less than this and the node gets deleted
#define LOWER_LIMIT_OBJECTS 10

//the initial cube has the size of a 256 meters, this should be sufficient
//for almost any map, should there be points outside of that we could either
//add one layer on top or just add them to the top layer
#define INITIAL_NODE_WIDTH 4000.0f


//TODO: mutex system that allowes to easily traverse and change the octree without having
//unnecessary lock states.
//each node would have to have at least 1 mutex for securing the list of OctreeMembers
//also each node would have at least 1 mutex for securing the list of subNodes.


//also: MULTIPLE READERS SINGLE WRITERS LOCK
//https://en.wikipedia.org/wiki/Readers%E2%80%93writer_lock
//std solution requires c++ 17
//http://en.cppreference.com/w/cpp/thread/shared_mutex
// also look up the difference between a mutex and a semaphore
//http://www.geeksforgeeks.org/mutex-vs-semaphore/

template <class T>
class OctreeNode;

template <class T>
class Octree;

template <class T>
class OctreeMember{
    friend Octree<T>;
private:
    Octree<T>* octree = nullptr;
    OctreeNode<T>* node = nullptr;
    Eigen::Vector3f pos;
    float radius;
public:
    OctreeMember(){

    }
    ~OctreeMember(){
        //std::cout << "OctreeMember destructor called" << std::endl;
        if(octree != nullptr && node!=nullptr){
            Octree<T> *tree = octree;
            tree->mutex.lock();
            node->removeObject(static_cast<T*>(this));
            tree->mutex.unlock();
        }
    }

    bool isInsertedInOctree(){
        return octree!= nullptr && node!=nullptr;
    }


    Eigen::Vector3f getPos(){return pos;}
    float getRadius(){return radius;}

    void setOctreeNode(OctreeNode<T>* node){
        this->node = node;
        assert(octree == nullptr || node != nullptr);
        if(node == nullptr && octree !=nullptr){
            assert(0);
        }
        //std::cout << "DEBUG octree: setting octree node" << std::endl;
    }
    void setOctree(Octree<T>* octree){this->octree = octree;}
    //Octree<T>* getOctree(){return octree;}

    /**
     * @brief setSphere
     * changes the position of the object and puts it into the right place of the map already
     * @param pos
     * @param radius
     */
    void setSphere(Eigen::Vector3f newPos,float newRadius){


        //set the values
        this->pos = newPos;
        this->radius = newRadius;

        if(octree!=nullptr && node!=nullptr){
            //lock the whole octree
            Octree<T>* tree = octree;
            octree->mutex.lock();

            //check if this object still is in the right octree node
            Eigen::Vector3f nodeCenter = node->getCenter();
            float nodeWidth_2 = node->getHalfWidth();
            if(!OctreeNode<T>::fitsWithinNode(newPos,newRadius,
                                 nodeCenter,nodeWidth_2)){

                //if  the object doesn't fit into this node anymore we have to relocate it
                //first it has to be romoved from the current node

                OctreeNode<T>* upperNode = node;
                OctreeNode<T>* oldNode = node;

                //actually the object will be completely removed from the octree
                // so node and octree references will be invalid
                std::shared_ptr<T> shared =
                        node->removeObject(static_cast<T*>(this),
                        false); //do it without changing the node....

                //assert(0); //removing object here could invalidate the upper node (which is this node right now)
                //TODO: Solution might be to prevent remove object in this context from removing upper nodes since we are just moving
                //the object


                //follow the octree upwards and put putInPlace it from there.
                while(upperNode!=nullptr){

                    Eigen::Vector3f nodeCenter = upperNode->getCenter();
                    float nodeWidth_2 = upperNode->getHalfWidth();
                    if(OctreeNode<T>::fitsWithinNode(newPos,newRadius,
                                         nodeCenter,nodeWidth_2)//if it fits into the node
                            || upperNode->parent == nullptr){//or we are at the uppermost node
                        //we found a node where this object would fit in
                        setOctree(tree);//readd the object to the octree
                        upperNode->putInPlace(shared); //this has to be a shared pointer
                        break;//get out of this loop

                    }
                    upperNode = upperNode->parent;

                }
                //cleanup if ths original node still makes sense and cleanup everything
                oldNode->checkAndShrink();
            }

            //unlock the mutex
            octree->mutex.unlock();
        }else{
            //std::cout << "DEBUG! (not a big problem usually! setting sphere of octree member even though it is not part of an octree yet" << std::endl;
        }





    }


    static bool checkVisibility(    const Eigen::Vector3f &center,
                                    const float &radius,
                                    const Eigen::Vector4f (&planes)[6],
                                    const Eigen::Matrix4f &_camPose,
                                    const float &maxDist);

    void setPos(Eigen::Vector3f pos){setSphere(pos,getRadius());}
    void setRadius(float radius){setSphere(getPos(),radius);}
    //TODO: somehow the octree node should be informed wheater this object gets changed or not
};

//TODO: check if this really is needed
template<class T>
class Octree{
    friend OctreeMember<T>;
private:
    OctreeNode<T> *rootNode;

    //actually there should be the possibilty to read from multiple threads
    //but only write from one
    std::mutex mutex;

public:
    Octree();
    ~Octree();

    /**
     * @brief getObjects
     * Given we have a camera position with intrinsics we test every object within the octree
     * if their bounding sphere is touching the frustum of the camera.
     *
     * @param camPos
     * @param intrinsics
     * @param maxDist
     * @return
     */
    std::vector<std::shared_ptr<T> > getObjects(Eigen::Matrix4f camPos,
                                                            Eigen::Vector4f intrinsics,
                                                            Eigen::Vector2f resolution,
                                                            float maxDist,
                                                            float dilateFrustum =0.0f);
    void addObject(std::shared_ptr<T> object){
        mutex.lock();
        rootNode->putInPlace(object);
        mutex.unlock();
    }
    void addObjects(std::vector<std::shared_ptr<T>> objects){
        mutex.lock();
        for(size_t i=0;i<objects.size();i++){
            rootNode->putInPlace(objects[i]);
        }
        mutex.unlock();
    }
    void removeObject(std::shared_ptr<OctreeMember<T>> object){
        mutex.lock();
        object->node->removeObject(object);
        mutex.unlock();
    }

    void removeObject(std::shared_ptr<T> object){
        mutex.lock();
        object->node->removeObject(object);
        mutex.unlock();
    }
};


template <class T>
class OctreeNode{
    friend OctreeMember<T>;
private:
    OctreeNode<T>* parent = nullptr;
    OctreeNode<T>* childs[8] = {nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr};
    Eigen::Vector3f center;
    float half_width;
public:
    OctreeNode(OctreeNode<T> *parent, Eigen::Vector3f center, float halfWidth);
    ~OctreeNode();

    //weak pointers is needed since we want to assemble lists of shared_ptr
    std::vector<std::weak_ptr<T>> objects;
    //std::vector<std::tuple<T*,std::weak_ptr<T>>> objects;


    Eigen::Vector3f getCenter(){ return center;}
    float getHalfWidth(){return half_width;}


    Eigen::Vector3f getCenterAtIndex(int ind);
    //end of line

    //splits the objects within this octree node up to where they belong
    void split();
    bool hasChildren(){
        for(size_t i=0;i<8;i++){
            if(childs[i]!=nullptr){
                return true;
            }
        }
        return false;
    }
    bool isFull(){
        return objects.size()>UPPER_LIMIT_OBJECTS;
    }
    bool isSparse(){
        return objects.size()<LOWER_LIMIT_OBJECTS && !hasChildren();
    }



    void appendVisibleObjects(std::vector<std::shared_ptr<T>> &visible,
                           const Eigen::Vector4f (&planes)[6],
                           const Eigen::Matrix4f &_camPose,
                           const float &maxDist,
                           const float &dilateFrustum=0.0f);


    //if all the subnodes have too few elements we delete them and store the remaining objects in the
    void eatChildren();//really necessary?

    //if this node does not have any children and very little objects move it to the node above
    void checkAndShrink();

    void removeNode(OctreeNode<T>* child){
        for(size_t i=0;i<8;i++){
            if(childs[i] == child){
                childs[i] = nullptr;
                return;
            }
        }
        assert(0);//removing a node failed
    }

    void addObject(std::shared_ptr<T> object){
        objects.push_back(object);
        object->setOctreeNode(this);
    }

    void putInPlace(std::shared_ptr<T> object);
    void removeObject(std::shared_ptr<T> object);


    std::shared_ptr<T> removeObject(T *object,bool checkAndShrink = true);


    bool fitsWithinNode(std::shared_ptr<T> object);

    bool childsAreSparse();

    bool childsAreSparseNoGrandchildren();

    static bool fitsWithinNode(Eigen::Vector3f center,
                               float radius, Eigen::Vector3f nodeCenter, float nodeWidth_2);


};



template<class T>
bool OctreeMember<T>::checkVisibility(    const Eigen::Vector3f &center,
                                      const float &radius,
                                      const Eigen::Vector4f (&planes)[6],
                                      const Eigen::Matrix4f &_camPose,
                                      const float &maxDist){
    for(size_t i=0;i<6;i++){

        Eigen::Vector4f p = Eigen::Vector4f(center[0],center[1],center[2],1);
        float distance = p.dot(planes[i]);
        if(distance > radius){
            return false;
        }
    }
    return true;


    //TODO: remove this old version of it
    /*
    //return true;
    Eigen::Vector4f p = _camPose*Eigen::Vector4f(center[0],center[1],center[2],1);
    if(p[2]-radius > maxDist || p[2]+radius < 0){
        return false;
    }
    p[3] = radius;
    for(size_t i=0;i<4;i++){
        float debugDist = planes[i].dot(p);
        if(planes[i].dot(p)>0){
            return false;
        }
    }


    return true;
    */

}




template<class T>
Octree<T>::Octree(){
    rootNode = new OctreeNode<T>(nullptr,Eigen::Vector3f(0,0,0),
                                 INITIAL_NODE_WIDTH/2.0f);
}


template<class T>
Octree<T>::~Octree(){
    delete rootNode;
}






template<class T>
std::vector<std::shared_ptr<T> > Octree<T>::getObjects( Eigen::Matrix4f camPos,
                                                        Eigen::Vector4f intrinsics,
                                                        Eigen::Vector2f resolution,
                                                        float maxDist,
                                                        float dilateFrustum)
{
    //The plan here is to transform all the poins into the camera frame to then test if they
    //are within the frustum


    float &fx=intrinsics[0];
    float &fy=intrinsics[1];
    float &cx=intrinsics[2];
    float &cy=intrinsics[3];
    float &rx=resolution[0];
    float &ry=resolution[1];
    //TODO: don't transform the center points but transform the plane equation according to the camera
    //list of objecs we are filling up here
    std::vector<std::shared_ptr<T>> objects;

    Eigen::Matrix4f _camPose = camPos.inverse();

    //create planes which cut the frustrum
    Eigen::Vector4f alpha(0,0,0,0);//-1);//0);//-1);
    Eigen::Vector4f planes[6]={
        Eigen::Vector4f(0,fy,-cy,0),//up
        Eigen::Vector4f(0,-fy,(cy-ry),0),//down
        Eigen::Vector4f(fx,0,-cx,0),//left
        Eigen::Vector4f(-fx,0,(cx-rx),0),//right
        Eigen::Vector4f(0,0,-1,0),//near plane
        Eigen::Vector4f(0,0,1,0)//far plane maxDist
    };
    /*
    for(size_t i=0;i<4;i++){
        planes[i] = planes[i].normalized() + alpha;
    }
    */
    for(size_t i = 0;i < 6; i++){
        planes[i].block<3,1>(0,0).normalize();
        planes[i].block<3,1>(0,0) = camPos.block<3,3>(0,0) * planes[i].block<3,1>(0,0);
        float dist = planes[i].block<3,1>(0,0).dot(camPos.block<3,1>(0,3));
        planes[i][3] = -dist;
    }
    planes[5][3] -= maxDist;



    mutex.lock();
    rootNode->appendVisibleObjects(objects,
                                   planes,
                                   _camPose,
                                   maxDist,
                                   dilateFrustum);
    mutex.unlock();
    return objects;

}




template<class T>
OctreeNode<T>::OctreeNode(OctreeNode<T> *parent, Eigen::Vector3f center, float halfWidth)
{
    this->parent = parent;
    this->center = center;
    this->half_width = halfWidth;
}

template<class T>
OctreeNode<T>::~OctreeNode()
{
    for(size_t i=0;i<8;i++){
        if(childs[i]!=nullptr){
            delete childs[i];
        }
    }
}
template<class T>
Eigen::Vector3f OctreeNode<T>::getCenterAtIndex(int ind)
{
    const Eigen::Vector3f vectors[8] ={
        Eigen::Vector3f( -1, -1, -1),//actually we could derive this from the first 3 bits of ind
        Eigen::Vector3f( -1, -1,  1),//(don't know if this would be shorter or faster)
        Eigen::Vector3f( -1,  1, -1),
        Eigen::Vector3f( -1,  1,  1),
        Eigen::Vector3f(  1, -1, -1),
        Eigen::Vector3f(  1, -1,  1),
        Eigen::Vector3f(  1,  1, -1),
        Eigen::Vector3f(  1,  1,  1)};
    return center + half_width*0.5f*vectors[ind];
}
template<class T>
void OctreeNode<T>::split()
{
    if(parent==nullptr){
        std::cout << "debug" << std::endl;
    }
    //std::cout << "TODO: WE HAVE TO MUTEX THIS" << std::endl;
    for(size_t i=0;i<objects.size();i++){
        std::shared_ptr<T> object = objects[i].lock();
        if(object == nullptr){
            assert(0);//in our scheme this should not happen. every object in this list should be valid
        }
        for(size_t j=0;j<8;j++){
            float sub_width_2=half_width/2.0f;
            Eigen::Vector3f sub_center=getCenterAtIndex(j);
            if(fitsWithinNode(object->getPos(),object->getRadius(),
                            sub_center,sub_width_2)){
                //test if the object would fit into this child node
                if(childs[j]==nullptr){
                    //if the child node doesn't exist yet we create one
                    childs[j] = new OctreeNode(this,sub_center,sub_width_2);
                }

                //add to the according child
                childs[j]->putInPlace(object);
                object->setOctreeNode(childs[j]);


                //and remove from this vector.....
                //TODO: find out if this is the way to go
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
                std::cout << "we really have to test if this is doing what "
                        "it is supposed to do" << std::endl;
#endif
                objects[i]= objects[objects.size()-1];
                objects.pop_back();//remove the last element
                break;
            }
        }
    }
}

template<class T>
void OctreeNode<T>::appendVisibleObjects(   std::vector<std::shared_ptr<T>> &visible,
                                            const Eigen::Vector4f (&planes)[6],
                                            const Eigen::Matrix4f &_camPose,
                                            const float &maxDist,
                                            const float &dilateFrustum){
    //first test if this node is visible
    if(!OctreeMember<T>::checkVisibility( this->center,
                                        this->half_width*sqrt(3)+dilateFrustum,
                                        planes,
                                        _camPose,
                                        maxDist)){
        //if not we just do nothing
        return;
    }

    //if the node is visible do the same for all the objects:
    for(size_t i=0;i<objects.size();i++){
        //check if the object is in place and append
        std::shared_ptr<T> object = objects[i].lock();
        if(object == nullptr){
            assert(0);
        }
        if(OctreeMember<T>::checkVisibility( object->getPos(),
                                           object->getRadius()+dilateFrustum,
                                           planes,
                                           _camPose,
                                           maxDist)){
            visible.push_back(object);

        }
    }


    //when done with that we go trough all the childs
    for(size_t i=0;i<8;i++){
        if(childs[i]!=nullptr){
            childs[i]->appendVisibleObjects(visible,
                                            planes,
                                            _camPose,
                                            maxDist,
                                            dilateFrustum);
        }
    }

}

template<class T>
void OctreeNode<T>::eatChildren()
{
    //std::cout << "WE HAVE TO MUTEX THIS" << std::endl;
    for(size_t i=0;i<8;i++){
        if(childs[i] != nullptr){
            objects.insert(objects.end(),childs[i]->objects.begin(),childs[i]->objects.end());
            if(childs[i]->hasChildren()){
                assert(0);//when deleting a child node we want them to be empty
            }
            delete childs[i];
            childs[i] = nullptr;
        }
    }
    if(isSparse() && parent!=nullptr){
        if(parent->isSparse() && parent->childsAreSparse()){
            parent->eatChildren();
        }

    }
}

template<class T>
void OctreeNode<T>::checkAndShrink(){

    if(isSparse() && childsAreSparseNoGrandchildren()){
        eatChildren();
    }
    if(parent != nullptr ){
        if(!hasChildren() && objects.size() == 0){
            parent->removeNode(this);
            parent->checkAndShrink();
            delete this;
            return;
        }
        if(!hasChildren() && isSparse()){
            parent->checkAndShrink();//check if we could be eaten by the parent
        }
    }
}

template<class T>
void OctreeNode<T>::putInPlace(std::shared_ptr<T> object)
{
    //std::cout << "WE HAVE TO MUTEX THIS" << std::endl;

    if(!hasChildren()){
        //only if the node doesn't have children and
        //is not full we wan't to add the object directly here
        if(!isFull()){
            //TODO: remove this debug shit
            T* obj = object.get();
            std::weak_ptr<T> wobj = object;
            addObject(object);
            return;
        }else{
            //if the node is full tough we want to relocate each object to where it belongs
            //we have to split it up and create new child nodes.
            split();
        }
    }
    float sub_width_2 = half_width/2.0f;
    for(size_t i=0;i<8;i++){
        Eigen::Vector3f sub_center=getCenterAtIndex(i);
        //test if the object would fit into this child node
        if(fitsWithinNode(object->getPos(),object->getRadius(),
                        sub_center,sub_width_2)){

            //create a new Node if necessary
            if(childs[i]==nullptr){
                //if the child node doesn't exist yet we create one
                childs[i] = new OctreeNode<T>(this,sub_center,sub_width_2);
            }
            childs[i]->putInPlace(object);
            //std::cout << "the objects node should be set" << std::endl;

            return;
        }

    }
    //if the patch doesn't fit into one of the patches we put it in.
    //(it propably is right at the border)
    addObject(object);
    if(!fitsWithinNode(object)){
        std::cout << "SERIOUS ERROR: actually this object doesn't fit in here either" << std::endl;
        assert(0);
    }
}


template <class T>
std::shared_ptr<T> OctreeNode<T>::removeObject(T *object,bool checkAndShrink){
    std::shared_ptr<T> objectShared = nullptr;
    //Iterate from end to beginning to remove the said object
    bool debugFoundObject = false;
    for(int i= objects.size()-1;i>=0;i--){
        std::shared_ptr<T> obj = objects[i].lock();
        T* raw = obj.get();
        if(raw == object || raw == nullptr){
            objectShared = obj;

            //remove object from list
            objects[i] = objects[objects.size()-1];//*objects.end();
            objects.pop_back();
            debugFoundObject = true;
            break;
        }
    }
    if(debugFoundObject==false){
        //TODO: find out why this really does not work out
        std::shared_ptr<T> obj = objects[objects.size()-1].lock();
        T* bla = obj.get();
        assert(0);
    }
    assert(debugFoundObject); //the object hast to be in the list. if not we are screwed.

    object->setOctree(nullptr);
    object->setOctreeNode(nullptr);


    if(!checkAndShrink){
        return objectShared;
    }
    //then we check if this node is low on objects
    if(isSparse() && !hasChildren()){
        //if it is and it has a parent node we check for the parent node
        if(parent!=nullptr){


            if(objects.size() == 0){
                //remove the nodes upwards
                parent->removeNode(this);
                parent->checkAndShrink();//running check and shrink also means that this node might not exist anymore
                delete this;
                return objectShared;
            }

            //check if the parent and its child are also sparse
            if(parent->childsAreSparseNoGrandchildren() && parent->isSparse()){
                //we combine the childs and put them in here.
                assert(0);//TODO: check out why this is never reached
                parent->eatChildren();
            }
        }
    }

    return objectShared;
}

template <class T>
void OctreeNode<T>::removeObject(std::shared_ptr<T> object)
{
    //std::cout << "WE HAVE TO MUTEX THIS" << std::endl;
    //first find object and remove it from vector
    removeObject(object.get());


}

template <class T>
bool OctreeNode<T>::fitsWithinNode(std::shared_ptr<T> object)
{
    return fitsWithinNode(object->getPos(),object->getRadius(),
                          center,half_width);
}

template <class T>
bool OctreeNode<T>::childsAreSparse()
{
    //std::cout << "WE HAVE TO MUTEX THIS! DO WE REALLY????" << std::endl;
    for(size_t i=0;i<8;i++){
        if(childs[i]!=nullptr){
            if(!childs[i]->isSparse()){
                return false;
            }
        }
    }
    return true;
}
template <class T>
bool OctreeNode<T>::childsAreSparseNoGrandchildren()
{
    //std::cout << "WE HAVE TO MUTEX THIS! DO WE REALLY????" << std::endl;
    for(size_t i=0;i<8;i++){
        if(childs[i]!=nullptr){
            if(!childs[i]->isSparse()){
                return false;
            }
            if(childs[i]->hasChildren()){
                return false;
            }
        }
    }
    return true;
}

template <class T>
bool OctreeNode<T>::fitsWithinNode(Eigen::Vector3f center,float radius,
                              Eigen::Vector3f nodeCenter, float nodeWidth_2)
{
    for(size_t i=0;i<3;i++){
        if(center[i] + radius > nodeCenter[i] + nodeWidth_2){
            return false;
        }
        if(center[i] - radius < nodeCenter[i] - nodeWidth_2){
            return false;
        }
    }
    return true;
}




#endif
