//! Octree for storing objects for rendering
/*! 
 * Provides the tools for creating an octree which can be used to store
 * objects with their respective positions and sizes. This can in turn be used
 * for things like obtaining the objects relevant for rendering or similar.
 *
 * \file   octree.h
 * \author Nikolaus Wagner
 * \author Simon Schreiberhuber
 * \date   6. 12. 2019
 *
 */

#ifndef FILE_OCTREE_H
#define FILE_OCTREE_H

#include <vector>
#include <memory>
#include <mutex>

#include <Eigen/Eigen>

#include <logging/include/logger.h>
#include <video_source/source.h>

using namespace std;
using namespace Eigen;

//! Everything concerning the octree used for storing objects for rendering
namespace octree {

// Forward declarations
class Object;
class Node;
class Octree;

//! A camera Frustum
/*! 
 *  This class defines a camera Frustum (as defined by 6 planes) and helper
 *  functions to determine whether or not an (octree-) Object is (fully/partly)
 *  visible for this Frustum.
 *
 *  \todo Split the visibility checks into fully & partly visible
 */
class Frustum {
public:
	//! Constructor for a camera Frustum
	/*!
	 *  Given all the necessary information, this function will calculate the
	 *  planes defining a camera Frustum.
	 *
	 *  \param pose The current camera pose
	 *  \param intrinsics The intrinsic camera parameters
	 *  \param resolution The number of pixels (width x height) the camera provides
	 *  \param max_depth The distance at which the Frustum will be truncated
	 */
	Frustum(Matrix4f pose, video::Intrinsics intrinsics, Vector2f resolution,
	        float max_depth);

	//! Visibility check for a spherical object
	/*!
	 *  Checks whether or not a spherical object is, at least partly, visible
	 *  for this Frustum.
	 *
	 *  \param center The center of the object to be checked
	 *  \param radius The radius of the object to be checked
	 *
	 *  \return Whether or not the object is visible
	 */
	bool contains(Vector3f center, float radius) const;

	//! Visibility check for an abstract octree Object
	/*!
	 *  Checks whether or not an abstract octree Object is, at least partly, 
	 *  visible for this Frustum.
	 *
	 *  \param object The octree Object to be checked
	 *
	 *  \return Whether or not the Object is visible
	 */
	bool contains(const Object *object) const;

	//! Visibility check for an octree Node
	/*!
	 *  Checks whether or not an octree Node is, at least partly, visible
	 *  for this Frustum.
	 *
	 *  \param node The octree Node to be checked
	 *
	 *  \return Whether or not the Node is visible
	 */
	bool contains(const Node *node) const;

private:
	Vector4f planes_[6]; //!< The 6 planes defining the frustum
};

//! An (usually abstract) object for insertion into an octree
/*! 
 *  For implementation purposes, one can simply inherit from this object
 *  and insert the subsequently created objects into an octree, where they
 *  will be treated as objects of this base class.
 *  These objects will be treated as spherical in the context of the octree.
 */
class Object : public enable_shared_from_this<Object> {
public:

	//! Constructor for the abstract object
	/*!
	 *  Creates an (abstract, spherical) object for insertion into the octree.
	 *
	 *  \param center The geometrical center of the object
	 *  \param radius The radius of the object
	 */
	Object(Vector3f center = Vector3f(0,0,0), float radius = 0.0f);

	virtual ~Object() = default;

	//! Checks whether the object geometrically fits inside an octree Node
	/*!
	 *  Check in all three dimensions whether the object fully fits inside an
	 *  octree Node.
	 *
	 *  \param node The Node for which to perform the check
	 *
	 *  \param Whether this object fits inside the Node
	 */
	bool fitsWithinNode(Node *node) const;

	//! Checks whether the Object is visible for a given Frustum
	/*!
	 *  Checks whether this Object is (at least partly) visible for a given
	 *  Frustum.
	 *
	 *  \param frustum The frustum for which to check
	 *
	 *  \return Whether the Object is visible
	 */
	inline bool isVisible(Frustum *frustum) const {
		return frustum->contains(this);
	}

	//! Move Object to another Node
	/*!
	 *  Try moving the Object to a given Node, if the Object does not fit, it will
	 *  be moved to the best fitting Node in the Octree instead.
	 *
	 *  \param target_node The desired Node where the Object shall be moved to
	 *  \param cleanup Whether or not to try to reduce the Octree after moving
	 *         the Object
	 *
	 *  \return The actual Node where the Object was finally inserted
	 */
	Node* moveTo(Node *target_node, bool cleanup = true);

	//! Set the bounding sphere of the Object
	/*!
	 *  Set the bounding sphere, i. e. the radius and center, of this Object
	 *  and subsequently check whether it still fits into the Node it had 
	 *  resided in so far. If it doesn't, try finding the new best fitting Node.
	 *
	 *	\param center The geometrical center of the new bounding sphere
	 *  \param radius The radius of the the new bounding sphere
	 *
	 *  \return The best fitting Node for this Object
	 */
	Node* setSphere(Vector3f center, float radius);

	//! Getter function for the host Node of this Object
	/*!
	 * \return The host Node this Object currently resides in
	 */
	inline Node* node() const {
		return host_node_;
	}

	//! Getter function for the geometrical center of this Object
	/*!
	 * \return The geometrical center of the bounding sphere of this Object
	 */
	inline Vector3f center() const {
		return center_;
	}

	//! Getter function for the radius of this Object
	/*!
	 * \return The radius of the bounding sphere of this Object
	 */
	inline float radius() const {
		return radius_;
	}


	void clear();

private:
	Node *host_node_; //!< The octree node where this object currently resides

	Vector3f center_; //!< The geometrical center of this object
	float radius_; //!< The radius of this object
};

//! A single Node of an Octree
/*! 
 *  This class defines one cubical Node of an Octree which has knowledge about
 *  its parent Node and child nodes, the Octree in which it resides 
 *  and can store Objects. 
 *  These Nodes are not supposed to be used on their own, but instead will be
 *  automatically instantiated/destructed as needed by the encapsulating
 *  Octree - class. 
 */
class Node {
friend Octree; //!< octree::Octree

public:

	//! Constructor
	/*!
	 *  \param center The geometrical center of this Node
	 *  \param width The side length of this Node
	 *  \param octree The Octree this node is part of
	 *  \param parent Direct parent of this Node
	 */
	Node(Vector3f center, float width, Octree* octree, Node* parent = nullptr);

	//! Destructor
	~Node() {
		for(uint8_t i = 0; i < 8; i++)
			delete children_[i];
	}

	//! Adds an Object to this Node
	/*!
	 *  Try adding an Object to this Node, if it fits. If it doesn't fit, the 
	 *  Object will be added into the best fitting Node instead.
	 *
	 *  \param object The Object to be added to this Node
	 *
	 *  \return The Node the Object was finally added to, or nullptr if adding failed
	 */
	Node* add(shared_ptr<Object> object);

	//! Remove an Object from this Node
	/*!
	 *  Try removing an Object from this Node. If the Object is not found in this
	 *  Node, try removing it from all children. If this also fails, do nothing.
	 *
	 *  \param object The Object to be removed from the Node
	 *  \param cleanup Whether or not to try merging after removing
	 *
	 *  \return The Node the Object was finally removed from, or nullptr if removing failed
	 */
	Node* remove(shared_ptr<Object> object, bool cleanup = true);

	//! Split this Node into 8 sub-Nodes
	/*!
	 *  Try splitting this Node up into 8 sub-Nodes, if it has enough Objects
	 *  and then distribute the fitting Objects among the new children.
	 */
	void split();

	//! Merge children with this node
	/*!
	 *  Try merging this Node with its children, leaving only this parent Node.
	 *  If there are any grandchildren or if there are too many objects to merge,
	 *  do nothing.
	 *  After merging this Node, try merging the next higher (parent) Node as
	 *  well.
	 */
	void merge();

	//! Get the visible objects of this node and its children
	/*!
	 *  Given a viewing Frustum, extract all visible Objects of this Node and
	 *  all its descendants and append them to a vector.
	 *
	 *  \tparam T The type of the Objects that have been inserted into the Octree
	 *  \param frustum The camera Frustum for which to extract the visible Objects
	 *  \param visible_objects The vector of visible Objects to which all the visible
	 *                         Objects of this Node and all its descendants will be appended
	 */
	template<class T>
	void getVisibleObjects(Frustum* frustum, 
	                       vector<shared_ptr<T>>* visible_objects) const {

		// Check if this node is visible at all
		if(!isVisible_(frustum))
			return;

		// Extract the visible objects of this node
		for(auto object : objects_) {
			if(object->isVisible(frustum)) {
				visible_objects->push_back(dynamic_pointer_cast<T>(object));
			}
		}

		// Extract the visible objects of the children
		if(hasChildren_()) {
			for(uint8_t i = 0; i < 8; i++) {
				children_[i]->getVisibleObjects<T>(frustum, visible_objects);
			}
		}

		return;
	}

	//! Getter function for the geometrical center of this Node
	/*!
	 * \return The geometrical center of this Node
	 */
	inline Vector3f center() const {
		return center_;
	}

	//! Getter function for the side length of this Node
	/*!
	 * \return The side length of this Node
	 */
	inline float width() const {
		return width_;
	}

private:

	//! Calculate the location of the geometrical centers of the (possible) children of this Node
	/*!
	 *  \return The geometrical centers of the (possible) children of this Node
	 */
	inline array<Vector3f, 8> childrenCenters_() {
		array<Vector3f, 8> children_centers = {
				center_ + (Vector3f(-width_, -width_, -width_) / 4),
				center_ + (Vector3f(-width_, -width_,  width_) / 4),
				center_ + (Vector3f(-width_,  width_, -width_) / 4),
				center_ + (Vector3f(-width_,  width_,  width_) / 4),
				center_ + (Vector3f( width_, -width_, -width_) / 4),
				center_ + (Vector3f( width_, -width_,  width_) / 4),
				center_ + (Vector3f( width_,  width_, -width_) / 4),
				center_ + (Vector3f( width_,  width_,  width_) / 4)};
		return children_centers;
	}

	//! Check whether this Node has any children
	/*!
	 *  \return Whether this Node has any children
	 */
	inline bool hasChildren_() const {
		for(uint8_t i = 0; i < 8; i++)
			if(children_[i] != nullptr)
				return true;

		return false;
	}

	//! Check whether this Node has a parent Node
	/*!
	 *  \return Whether this Node has a parent Node
	 */
	inline bool isRoot_() const {
		return (parent_ == nullptr);
	}

	//! Check whether this Node could be merged with its siblings into the parent Node
	/*!
	 *  \return Whether this Node has few enough Object to be merged and whether it has any child Nodes
	 */
	inline bool isSparse_() const {
		return (objects_.size() < min_num_objects_) && !hasChildren_();
	}

	//! Check whether this Node contains enough Objects for it to be split up
	/*!
	 *  \return Whether this Node has more Objects than \ref max_num_objects_, thus
	 *          whether or not it should be split up into sub-Nodes.
	 */
	inline bool isFull_() const {
		// cannot be full if has children, as that would mess with our concept of storing too large objects in the parent node
		return (objects_.size() >= max_num_objects_) && !hasChildren_(); 
	}

	//! Check whether this Node is (partially) visible
	/*!
	 *  Given a camera Frustum, check whether or not this Node is (at least partially)
	 *  visible.
	 *
	 *  \param frustum The camera Frustum for which to check the visibility
	 *
	 *  \return Whether this Node is visible
	 */
	inline bool isVisible_(Frustum *frustum) const {
		return frustum->contains(this);
	}

	const int min_num_objects_; //!< If the Node contains fewer Objects than this, we will try to merge it with its siblings
	const int max_num_objects_; //!< If the Node contains more Objects than this, we will try to split it up

	Octree *octree_; //!< The Octree in which this Node resides

	Node *parent_; //!< The direct parent of this Node
	Node *children_[8]; //!< The 8 direct children of a Node

	Vector3f center_; //!< The geometrical center of a node
	float width_; //!< The width of this Node

	vector<shared_ptr<Object>> objects_; //!< The Objects stored in this Node

};

//! Meta-class defining an octree
/*! 
 *  This class encapsulates an octree consisting of separate nodes.
 *  The main purpose of this class is to enable multithreading by encapsulating
 *  all the relevant functions of the individual octree-nodes with mutexes.
 */
class Octree {
friend Node; //!< octree::Node

public:

	//! Constructor
	/*!
	 *  \param center The geometrical center of the Octree's root Node
	 *  \param width The side length of the Octree's root Node
	 */
	Octree(Vector3f center = Vector3f(0,0,0), float width = 2048.0f)
			: max_width_(1024.0f),
			  root_node_(new Node(center, width, this)) {

	}

	//! Destructor
	~Octree() {
		delete root_node_;
	}

	//! Add an object to the octree
	/*!
	 *  Traverse the octree top down and try to insert an object into the smallest
	 *  possible descendant.
	 *  If the root node is not large enough to store the object, the octree
	 *  is recursively expanded upwards until the object can be inserted.
	 *
	 *  \param[in] object Pointer to the object to be inserted into the octree
	 *
	 *  \return Pointer to the node where the object was finally inserted
	 */
	Node* add(shared_ptr<Object> object);

	//! Remove an Object from the Octree
	/*!
	 *  Try removing an Object from the root Node, which will recursively lead
	 *  to an attempt of finding the Object in all of the descendant Nodes and removing
	 *  it if it is found.
	 *
	 *  \param object The Object to be removed from the Octree
	 *
	 *  \return The Node from which the Object was finally removed
	 */
	inline Node* remove(shared_ptr<Object> object) {
		mutex_.lock();
		auto host = root_node_->remove(object);
		mutex_.unlock();
	}

	//! Get the visible Objects of this Octree
	/*!
	 *  Given a viewing Frustum, extract all visible Objects of this Octree
	 *  and append them to a vector.
	 *
	 *  \tparam T The type of the Objects that have been inserted into the Octree
	 *  \param frustum The camera Frustum for which to extract the visible Objects
	 *  \param visible_objects The vector of visible Objects to which all the visible
	 *                         Objects will be appended
	 */
	template<class T>
	inline void getVisibleObjects(Frustum* frustum, 
	                              vector<shared_ptr<T>>* visible_objects) {
		mutex_.lock();
		root_node_->getVisibleObjects(frustum, visible_objects);
		mutex_.unlock();
	}

	//! Getter function for the geometrical center of this Octree
	/*!
	 * \return The geometrical center of this Octree
	 */
	inline Vector3f center() {
		mutex_.lock();
		return root_node_->center();
		mutex_.unlock();
	}

	//! Getter function for the side length of this Octree
	/*!
	 * \return The side length of this Octree
	 */
	inline float width() {
		mutex_.lock();
		return root_node_->width();
		mutex_.unlock();
	}

private:
	Node *root_node_; //!< The uppermost Node of the Octree

	mutex mutex_; //!< Mutex for enabling multithreaded access of the Octree

	float max_width_; //!< The maximum side lenght of the Octree, i. e. the root Node
};

} // namespace octree

#endif // FILE_OCTREE_H