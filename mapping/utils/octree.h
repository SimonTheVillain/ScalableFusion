//https://geidav.wordpress.com/2014/08/18/advanced-octrees-2-node-representations/
//https://geidav.wordpress.com/2014/11/18/advanced-octrees-3-non-static-octrees/
#ifndef FILE_OCTREE_H
#define FILE_OCTREE_H

#include <vector>
#include <memory>
#include <mutex>
#include <iostream>

#include <Eigen/Eigen>

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

using namespace std;
using namespace Eigen;

template <class T>
class OctreeNode;
 
template <class T>
class Octree;

template <class T>
class OctreeMember {
	friend Octree<T>;

public:

	OctreeMember()
			: octree_(nullptr),
			  node_(nullptr) {
	}

	~OctreeMember() {
		if(octree_ != nullptr && node_ != nullptr) {
			Octree<T> *tree = octree_;
			tree->mutex_.lock();
			node_->removeObject(static_cast<T*>(this));
			tree->mutex_.unlock();
		}
	}

	bool isInsertedInOctree() {
		return octree_ != nullptr && node_ != nullptr;
	}

	Vector3f getPos() {
		return pos_;
	}

	float getRadius() {
		return radius_;
	}

	void setOctreeNode(OctreeNode<T> *node) {
		node_ = node;
		assert(octree_ == nullptr || node_ != nullptr);
		if(node_ == nullptr && octree_ != nullptr) {
			assert(0);
		}
	}

	void setOctree(Octree<T> *octree) {
		octree_ = octree;
	}

	/**
	 * @brief setSphere
	 * changes the position of the object and puts it into the right place of the map already
	 * @param pos
	 * @param radius
	 */
	void setSphere(Vector3f new_pos, float new_radius) {

		//set the values
		pos_ = new_pos;
		radius_ = new_radius;

		if(octree_ != nullptr && node_ != nullptr) {
			//lock the whole octree
			Octree<T> *tree = octree_;
			octree_->mutex_.lock();

			//check if this object still is in the right octree node
			Vector3f node_center = node_->getCenter();
			float node_width_2 = node_->getHalfWidth();
			if(!OctreeNode<T>::fitsWithinNode(new_pos, new_radius, node_center, 
			                                  node_width_2)) {

				//if  the object doesn't fit into this node anymore we have to relocate it
				//first it has to be romoved from the current node
				OctreeNode<T> *upper_node = node_;
				OctreeNode<T> *old_node = node_;

				//actually the object will be completely removed from the octree
				// so node and octree references will be invalid
				shared_ptr<T> shared = node_->removeObject(static_cast<T*>(this), 
				                                           false); //do it without changing the node....

				//assert(0); //removing object here could invalidate the upper node (which is this node right now)
				//TODO: Solution might be to prevent remove object in this context from removing upper nodes since we are just moving
				//the object

				//follow the octree upwards and put putInPlace it from there.
				while(upper_node != nullptr) {
					Vector3f node_center = upper_node->getCenter();
					float node_width_2 = upper_node->getHalfWidth();
					if(OctreeNode<T>::fitsWithinNode(new_pos, new_radius, node_center,
					                                 node_width_2) ||//if it fits into the node
					   upper_node->parent_ == nullptr) {//or we are at the uppermost node
						//we found a node where this object would fit in
						setOctree(tree);//readd the object to the octree
						upper_node->putInPlace(shared); //this has to be a shared pointer
						break;
					}
					upper_node = upper_node->parent_;
				}
				//cleanup if ths original node still makes sense and cleanup everything
				old_node->checkAndShrink();
			}
			//unlock the mutex
			octree_->mutex_.unlock();
		}
	}

	static bool checkVisibility(const Vector3f &center, const float &radius,
	                            const Vector4f (&planes)[6], 
	                            const Matrix4f &cam_pose, const float &max_dist);

	void setPos(Vector3f pos) {
		setSphere(pos, getRadius());
	}
	void setRadius(float radius) {
		setSphere(getPos(), radius);
	}
	//TODO: somehow the octree node should be informed wheater this object gets changed or not

private:
	Octree<T> *octree_;
	OctreeNode<T> *node_;
	Vector3f pos_;
	float radius_;

};

//TODO: check if this really is needed
template<class T>
class Octree {
	friend OctreeMember<T>;

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
	vector<shared_ptr<T> > getObjects(Matrix4f cam_pos, Vector4f intrinsics,
	                                  Vector2f resolution, float max_dist,
	                                  float dilate_frustum = 0.0f);

	void addObject(shared_ptr<T> object) {
		mutex_.lock();
		root_node_->putInPlace(object);
		mutex_.unlock();
	}

	void addObjects(vector<shared_ptr<T>> objects) {
		mutex_.lock();
		for(size_t i = 0; i < objects.size(); i++) {
			root_node_->putInPlace(objects[i]);
		}
		mutex_.unlock();
	}

	void removeObject(shared_ptr<OctreeMember<T>> object) {
		mutex_.lock();
		object->node_->removeObject(object);
		mutex_.unlock();
	}

	void removeObject(shared_ptr<T> object) {
		mutex_.lock();
		object->node_->removeObject(object);
		mutex_.unlock();
	}

private:
	
	OctreeNode<T> *root_node_;

	//actually there should be the possibilty to read from multiple threads
	//but only write from one
	mutex mutex_;

};


template <class T>
class OctreeNode {
	friend OctreeMember<T>;

public:

	OctreeNode(OctreeNode<T> *parent, Vector3f center, float half_width);

	~OctreeNode();

	//weak pointers is needed since we want to assemble lists of shared_ptr
	vector<weak_ptr<T>> objects;

	Vector3f getCenter() {
		return center_;
	}

	float getHalfWidth() {
		return half_width_;
	}

	Vector3f getCenterAtIndex(int ind);

	//splits the objects within this octree node up to where they belong
	void split();

	bool hasChildren() {
		for(size_t i = 0; i < 8; i++) {
			if(children_[i] != nullptr) {
				return true;
			}
		}
		return false;
	}

	bool isFull() {
		return objects.size() > UPPER_LIMIT_OBJECTS;
	}

	bool isSparse() {
		return (objects.size() < LOWER_LIMIT_OBJECTS) && !hasChildren();
	}

	void appendVisibleObjects(vector<shared_ptr<T>> &visible, 
	                          const Vector4f (&planes)[6],
	                          const Matrix4f &cam_pose,
	                          const float &max_dist,
	                          const float &dilate_frustum = 0.0f);

	//if all the subnodes have too few elements we delete them and store the remaining objects in the
	void eatChildren();//really necessary?

	//if this node does not have any children and very little objects move it to the node above
	void checkAndShrink();

	void removeNode(OctreeNode<T> *child) {
		for(size_t i = 0; i < 8; i++) {
			if(children_[i] == child) {
				children_[i] = nullptr;
				return;
			}
		}
		assert(0);//removing a node failed
	}

	void addObject(shared_ptr<T> object) {
		objects.push_back(object);
		object->setOctreeNode(this);
	}

	void putInPlace(shared_ptr<T> object);

	void removeObject(shared_ptr<T> object);

	shared_ptr<T> removeObject(T *object, bool check_and_shrink = true);

	bool fitsWithinNode(shared_ptr<T> object);

	bool childrenAreSparse();

	bool childrenAreSparseNoGrandchildren();

	static bool fitsWithinNode(Vector3f center, float radius, 
	                           Vector3f node_center, float node_width_2);

private:

	OctreeNode<T> *parent_ = nullptr;
	OctreeNode<T> *children_[8];
	Vector3f center_;
	float half_width_;

};

template<class T>
bool OctreeMember<T>::checkVisibility(const Vector3f &center, 
                                      const float &radius,
                                      const Vector4f (&planes)[6],
                                      const Matrix4f &cam_pose,
                                      const float &max_dist) {
	for(size_t i = 0; i < 6; i++) {
		Vector4f p = Vector4f(center[0], center[1], center[2], 1);
		float distance = p.dot(planes[i]);
		if(distance > radius) {
			return false;
		}
	}
	return true;
}

template<class T>
Octree<T>::Octree()
		: root_node_(new OctreeNode<T>(nullptr, Vector3f(0, 0, 0), 
		                               INITIAL_NODE_WIDTH / 2.0f)) {
}

template<class T>
Octree<T>::~Octree() {
	delete root_node_;
}

template<class T>
vector<shared_ptr<T> > Octree<T>::getObjects(Matrix4f cam_pos, 
                                             Vector4f intrinsics,
                                             Vector2f resolution,
                                             float max_dist,
                                             float dilate_frustum) {
	//The plan here is to transform all the poins into the camera frame to then test if they
	//are within the frustum
	float &fx = intrinsics[0];
	float &fy = intrinsics[1];
	float &cx = intrinsics[2];
	float &cy = intrinsics[3];
	float &rx = resolution[0];
	float &ry = resolution[1];

	//TODO: don't transform the center points but transform the plane equation according to the camera
	//list of objecs we are filling up here
	vector<shared_ptr<T>> objects;

	Matrix4f cam_pose = cam_pos.inverse();

	//create planes which cut the frustrum
	Vector4f alpha(0, 0, 0, 0);
	Vector4f planes[6] = {Vector4f(  0,  fy,     -cy, 0),  //up
	                      Vector4f(  0, -fy, cy - ry, 0),  //down
	                      Vector4f( fx,   0,     -cx, 0),  //left
	                      Vector4f(-fx,   0, cx - rx, 0),  //right
	                      Vector4f(  0,   0,      -1, 0),  //near plane
	                      Vector4f(  0,   0,       1, 0)}; //far plane maxDist

	for(size_t i = 0; i < 6; i++) {
		planes[i].block<3, 1>(0, 0).normalize();
		planes[i].block<3, 1>(0, 0) = 
				cam_pos.block<3, 3>(0, 0) * planes[i].block<3, 1>(0, 0);
		float dist = planes[i].block<3, 1>(0, 0).dot(cam_pos.block<3, 1>(0, 3));
		planes[i][3] = -dist;
	}
	planes[5][3] -= max_dist;

	mutex_.lock();
	root_node_->appendVisibleObjects(objects, planes, cam_pose, max_dist,
	                                 dilate_frustum);
	mutex_.unlock();
	return objects;
}

template<class T>
OctreeNode<T>::OctreeNode(OctreeNode<T> *parent, Vector3f center, 
                          float half_width)
		: parent_(parent),
		  children_{nullptr, nullptr, nullptr, nullptr,
		            nullptr, nullptr, nullptr, nullptr},
		  center_(center),
		  half_width_(half_width) {
}

template<class T>
OctreeNode<T>::~OctreeNode() {
	for(size_t i = 0; i < 8; i++) {
		if(children_[i] != nullptr) {
			delete children_[i];
		}
	}
}

template<class T>
Vector3f OctreeNode<T>::getCenterAtIndex(int ind) {
	const Vector3f vectors[8] = {Vector3f( -1, -1, -1),//actually we could derive this from the first 3 bits of ind
	                             Vector3f( -1, -1,  1),//(don't know if this would be shorter or faster)
	                             Vector3f( -1,  1, -1),
	                             Vector3f( -1,  1,  1),
	                             Vector3f(  1, -1, -1),
	                             Vector3f(  1, -1,  1),
	                             Vector3f(  1,  1, -1),
	                             Vector3f(  1,  1,  1)};

	return center_ + half_width_ * 0.5f * vectors[ind];
}

template<class T>
void OctreeNode<T>::split() {
	if(parent_ == nullptr) {
		cout << "debug" << endl;
	}
	//cout << "TODO: WE HAVE TO MUTEX THIS" << endl;
	for(size_t i = 0; i < objects.size(); i++) {
		shared_ptr<T> object = objects[i].lock();
		if(object == nullptr) {
			assert(0);//in our scheme this should not happen. every object in this list should be valid
		}
		for(size_t j = 0; j < 8; j++) {
			float sub_width_2 = half_width_ / 2.0f;
			Vector3f sub_center = getCenterAtIndex(j);
			if(fitsWithinNode(object->getPos(), object->getRadius(), 
			                  sub_center, sub_width_2)) {
				//test if the object would fit into this child node
				if(children_[j] == nullptr) {
					//if the child node doesn't exist yet we create one
					children_[j] = new OctreeNode(this, sub_center, sub_width_2);
				}

				//add to the according child
				children_[j]->putInPlace(object);
				object->setOctreeNode(children_[j]);

				//and remove from this vector.....
				//TODO: find out if this is the way to go
				#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
				cout << "we really have to test if this is doing what "
				        "it is supposed to do" << endl;
				#endif
				objects[i] = objects[objects.size() - 1];
				objects.pop_back();//remove the last element
				break;
			}
		}
	}
}

template<class T>
void OctreeNode<T>::appendVisibleObjects(vector<shared_ptr<T>> &visible,
                                         const Vector4f (&planes)[6],
                                         const Matrix4f &cam_pose,
                                         const float &max_dist,
                                         const float &dilate_frustum){
	//first test if this node is visible
	if(!OctreeMember<T>::checkVisibility(center_,
	                                     half_width_ * sqrt(3) + dilate_frustum,
	                                     planes, cam_pose, max_dist)) {
		//if not we just do nothing
		return;
	}

	//if the node is visible do the same for all the objects:
	for(size_t i = 0; i < objects.size(); i++) {
		//check if the object is in place and append
		shared_ptr<T> object = objects[i].lock();
		if(object == nullptr) {
			assert(0);
		}
		if(OctreeMember<T>::checkVisibility(object->getPos(),
		                                    object->getRadius() + dilate_frustum,
		                                    planes, cam_pose, max_dist)) {
			visible.push_back(object);
		}
	}

	//when done with that we go trough all the childs
	for(size_t i = 0; i < 8; i++) {
		if(children_[i] != nullptr) {
		   children_[i]->appendVisibleObjects(visible, planes, cam_pose, max_dist,
		                                      dilate_frustum);
		}
	}
}

template<class T>
void OctreeNode<T>::eatChildren() {
	//cout << "WE HAVE TO MUTEX THIS" << endl;
	for(size_t i = 0; i < 8; i++) {
		if(children_[i] != nullptr) {
			objects.insert(objects.end(), children_[i]->objects.begin(), 
			               children_[i]->objects.end());
			if(children_[i]->hasChildren()) {
				assert(0);//when deleting a child node we want them to be empty
			}
			delete children_[i];
			children_[i] = nullptr;
		}
	}
	if(isSparse() && parent_ != nullptr) {
		if(parent_->isSparse() && parent_->childrenAreSparse()) {
			parent_->eatChildren();
		}
	}
}

template<class T>
void OctreeNode<T>::checkAndShrink() {
	if(isSparse() && childrenAreSparseNoGrandchildren()) {
		eatChildren();
	}
	if(parent_ != nullptr) {
		if(!hasChildren() && objects.size() == 0) {
			parent_->removeNode(this);
			parent_->checkAndShrink();
			delete this;
			return;
		}
		if(!hasChildren() && isSparse()) {
			parent_->checkAndShrink();//check if we could be eaten by the parent
		}
	}
}

template<class T>
void OctreeNode<T>::putInPlace(shared_ptr<T> object) {
	//cout << "WE HAVE TO MUTEX THIS" << endl;
	if(!hasChildren()) {
		//only if the node doesn't have children and
		//is not full we wan't to add the object directly here
		if(!isFull()) {
			//TODO: remove this debug shit
			T* obj = object.get();
			weak_ptr<T> wobj = object;
			addObject(object);
			return;
		} else {
			//if the node is full tough we want to relocate each object to where it belongs
			//we have to split it up and create new child nodes.
			split();
		}
	}
	float sub_width_2 = half_width_ / 2.0f;
	for(size_t i = 0; i < 8; i++) {
		Vector3f sub_center=getCenterAtIndex(i);
		//test if the object would fit into this child node
		if(fitsWithinNode(object->getPos(),object->getRadius(), 
		                  sub_center, sub_width_2)) {

			//create a new Node if necessary
			if(children_[i] == nullptr) {
				//if the child node doesn't exist yet we create one
				children_[i] = new OctreeNode<T>(this, sub_center, sub_width_2);
			}
			children_[i]->putInPlace(object);

			return;
		}
	}

	//if the patch doesn't fit into one of the patches we put it in.
	//(it propably is right at the border)
	addObject(object);
	if(!fitsWithinNode(object)) {
		cout << "SERIOUS ERROR: actually this object doesn't fit in here either" << endl;
		assert(0);
	}
}

template <class T>
shared_ptr<T> OctreeNode<T>::removeObject(T *object, bool check_and_shrink) {
	shared_ptr<T> object_shared = nullptr;
	//Iterate from end to beginning to remove the said object
	bool debug_found_object = false;
	for(int i = objects.size() - 1; i >= 0; i--) {
		shared_ptr<T> obj = objects[i].lock();
		T *raw = obj.get();
		if(raw == object || raw == nullptr) {
			object_shared = obj;
			//remove object from list
			objects[i] = objects[objects.size() - 1];
			objects.pop_back();
			debug_found_object = true;
			break;
		}
	}
	if(debug_found_object == false) {
		//TODO: find out why this really does not work out
		shared_ptr<T> obj = objects[objects.size() - 1].lock();
		assert(0);
	}
	assert(debug_found_object); //the object hast to be in the list. if not we are screwed.

	object->setOctree(nullptr);
	object->setOctreeNode(nullptr);

	if(!check_and_shrink) {
		return object_shared;
	}
	//then we check if this node is low on objects
	if(isSparse() && !hasChildren()) {
		//if it is and it has a parent node we check for the parent node
		if(parent_ != nullptr) {
			if(!objects.empty()) {
				//remove the nodes upwards
				parent_->removeNode(this);
				parent_->checkAndShrink();//running check and shrink also means that this node might not exist anymore
				delete this;
				return object_shared;
			}
			//check if the parent and its child are also sparse
			if(parent_->childrenAreSparseNoGrandchildren() && parent_->isSparse()) {
				//we combine the children and put them in here.
				assert(0);//TODO: check out why this is never reached
				parent_->eatChildren();
			}
		}
	}
	return object_shared;
}

template <class T>
void OctreeNode<T>::removeObject(shared_ptr<T> object) {
	//cout << "WE HAVE TO MUTEX THIS" << endl;
	//first find object and remove it from vector
	removeObject(object.get());
}

template <class T>
bool OctreeNode<T>::fitsWithinNode(shared_ptr<T> object) {
	return fitsWithinNode(object->getPos(), object->getRadius(), center_, 
	                      half_width_);
}

template <class T>
bool OctreeNode<T>::childrenAreSparse() {
	//cout << "WE HAVE TO MUTEX THIS! DO WE REALLY????" << endl;
	for(size_t i = 0; i < 8; i++) {
		if(children_[i] != nullptr) {
			if(!children_[i]->isSparse()) {
				return false;
			}
		}
	}
	return true;
}

template <class T>
bool OctreeNode<T>::childrenAreSparseNoGrandchildren() {
	//cout << "WE HAVE TO MUTEX THIS! DO WE REALLY????" << endl;
	for(size_t i = 0; i < 8; i++) {
		if(children_[i] != nullptr) {
			if(!children_[i]->isSparse()) {
				return false;
			}
			if(children_[i]->hasChildren()){
				return false;
			}
		}
	}
	return true;
}

template <class T>
bool OctreeNode<T>::fitsWithinNode(Vector3f center, float radius,
                                   Vector3f node_center, float node_width_2) {
	for(size_t i = 0; i < 3; i++) {
		if(center[i] + radius > node_center[i] + node_width_2) {
			return false;
		}
		if(center[i] - radius < node_center[i] - node_width_2) {
			return false;
		}
	}
	return true;
}

#endif // FILE_OCTREE_H
