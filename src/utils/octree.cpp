//! Octree for storing objects for rendering
/*! 
 * Provides the tools for creating an octree which can be used to store
 * objects with their respective positions and sizes. This can in turn be used
 * for things like obtaining the objects relevant for rendering or similar.
 *
 * \file   octree.cpp
 * \author Nikolaus Wagner
 * \author Simon Schreiberhuber
 * \date   6. 12. 2019
 *
 */

#include <utils/octree.h>

#include <iostream>

namespace octree {

Frustum::Frustum(Matrix4f pose, video::Intrinsics intrinsics, 
                 Vector2f resolution, float max_depth) {
	float fx = intrinsics.fx;
	float fy = intrinsics.fy;
	float cx = intrinsics.cx;
	float cy = intrinsics.cy;
	float rx = resolution[0];
	float ry = resolution[1];

	planes_[0] = Vector4f(  0,  fy,     -cy, 0); // up
	planes_[1] = Vector4f(  0, -fy, cy - ry, 0); // down
	planes_[2] = Vector4f( fx,   0,     -cx, 0); // left
	planes_[3] = Vector4f(-fx,   0, cx - rx, 0); // right
	planes_[4] = Vector4f(  0,   0,      -1, 0); // near
	planes_[5] = Vector4f(  0,   0,       1, 0); // far
	                                            // wherever you are
	for(uint8_t i = 0; i < 6; i++) {
		planes_[i].block<3, 1>(0, 0).normalize();
		planes_[i].block<3, 1>(0, 0) = pose.block<3, 3>(0, 0) * planes_[i].block<3, 1>(0, 0);
		float dist = planes_[i].block<3, 1>(0, 0).dot(pose.block<3, 1>(0, 3));
		planes_[i][3] = -dist;
	}
	planes_[5][3] -= max_depth;
}

bool Frustum::contains(Vector3f center, float radius) const {
	Vector4f center_hmg(center(0), center(1), center(2), 1);
	for(auto plane : planes_) {
		float dist = center_hmg.dot(plane);
		if(dist > radius)
			return false;
	}
	return true;
}

bool Frustum::contains(const Object* object) const {
	return contains(object->center(), object->radius());
}

bool Frustum::contains(const Node* node) const {
	return contains(node->center(), (node->width() / 2) * sqrt(3));
}

Object::Object(Vector3f center, float radius) 
		: center_(center),
		  radius_(radius),
		  host_node_(nullptr) {

}

bool Object::fitsWithinNode(Node* node) const {
	for(uint8_t i = 0; i < 3; i++) {
		if((center_[i] + radius_) > (node->center()[i] + (node->width() / 2)))
			return false;

		if((center_[i] - radius_) < (node->center()[i] - (node->width() / 2)))
			return false;

	}
	return true;
}

Node* Object::moveTo(Node *target_node, bool cleanup) {
	stringstream ss;
	ss << "Try moving object " << this << " from " << host_node_ << " to " 
	   << target_node;
	LOG(logging::Level::DEBUG, ss.str());

	if(host_node_)
		host_node_->remove(shared_from_this(), cleanup);

	if(target_node)
		host_node_ = target_node->add(shared_from_this());

	return host_node_;
}

Node* Object::setSphere(Vector3f center, float radius) {
	stringstream ss;
	ss << "Changing object " << this << " from " << center_(0) << ", " 
	   << center_(1) << ", " << center_(2) << ", radius = " << radius_ 
	   << " to " << center(0) << ", " << center(1) << ", " << center(2) 
	   << ", radius = " << radius;
	LOG(logging::Level::DEBUG, ss.str());

	center_ = center;
	radius_ = radius;

	return moveTo(host_node_);
}


Node::Node(Vector3f center, float width, Octree* octree, Node* parent) 
		: octree_(octree),
		  parent_(parent),
		  children_{nullptr, nullptr, nullptr, nullptr,
		            nullptr, nullptr, nullptr, nullptr},
		  min_num_objects_(10),
		  max_num_objects_(10 * min_num_objects_),
		  center_(center),
		  width_(width) {
	stringstream ss;
	ss << "Creating node " << this << " with parent " << parent
	   << " at " << center_(0) << ", " << center_(1) << ", " << center_(2) 
	   << " of width " << width_;
	LOG(logging::Level::DEBUG, ss.str());
}

Node* Node::add(shared_ptr<Object> object) {

	// Check if this node can story any more objects
	if(isFull_())
		split();

	// Try to fit into smallest possible descendant
	if(hasChildren_()) {
		for(uint8_t i = 0; i < 8; i++) {
			if(object->fitsWithinNode(children_[i])) {
				return children_[i]->add(object);
			}
		}
	}

	// Try to add to this node 
	if(object->fitsWithinNode(this)) { 
		objects_.push_back(object);
		stringstream ss;
		ss << "Added obect " << object << " to node " << this;
		LOG(logging::Level::DEBUG, ss.str());
		return this;
	}

	// Resort to fitting it into next larger parent node
	if(!isRoot_()) {
		return parent_->add(object);
	} else {
		LOG(logging::Level::ERROR, "Tried accessing parent from root node");
		assert(0);
	}

}

Node* Node::remove(shared_ptr<Object> object, bool cleanup) {
	// Try removing from this node
	for(uint32_t i = 0; i < objects_.size(); i++) {
		if(object == objects_[i]) {
			// Remove the object from this node
			objects_[i] = objects_[objects_.size() - 1];
			objects_.pop_back();
			
			// Clean up after
			if(cleanup)
				merge();

			return this;
		}
	}

	// Try removing from children
	if(hasChildren_())
		for(uint8_t i = 0; i < 8; i++)
			return children_[i]->remove(object);

	// Failed in finding object in node and all descendants
	LOG(logging::Level::WARNING, "Tried removing a non-existent object from a node");
	return nullptr;
}

void Node::split() {

	// Check if node is already split
	if(hasChildren_())
		return;

	// Check if splitting is necessary
	if(!isFull_())
		return;

	stringstream ss;
	ss << "Splitting up node " << this;
	LOG(logging::Level::DEBUG, ss.str());

	// Calculate locations of new children
	array<Vector3f, 8> children_centers = childrenCenters_();

	for(uint8_t i = 0; i < 8; i++) {
		// Create new children
		children_[i] = new Node(children_centers[i], width_ / 2, octree_, this);
		stringstream ss;
		ss << "Creating node " << children_[i] << " as child of " << this;
		LOG(logging::Level::DEBUG, ss.str());

		// Distribute objects among the children
		for(auto object : objects_) {
			// Move object to children if it fits, otherwise leave it in this node
			if(object->fitsWithinNode(children_[i])) {
				object->moveTo(children_[i], false);
			}
		}
	}

	return;
}

void Node::merge() {

	uint32_t num_objects_children = 0;

	// Check if there is anything to merge
	if(!hasChildren_())
		return;

	// Check if children can be merged
	for(uint8_t i = 0; i < 8; i++) {
		if(!children_[i]->isSparse_())
			return;

		num_objects_children += children_[i]->objects_.size();
	}

	// Check if merging would leave this node with too many elements
	if(num_objects_children + objects_.size() > max_num_objects_)
		return;

	// Finally, merge
	for(uint8_t i = 0; i < 8; i++) {
		objects_.insert(objects_.end(), 
		                children_[i]->objects_.begin(),
		                children_[i]->objects_.end());

		delete children_[i];
		children_[i] = nullptr;
	}

	// Now try merging parent
	if(!isRoot_()) 
		parent_->merge();

	return;
}

Node* Octree::add(shared_ptr<Object> object) {
	mutex_.lock();
	if(object->fitsWithinNode(root_node_)) {
		auto host = object->moveTo(root_node_);
		mutex_.unlock();
		return host;
	} else {
		LOG(logging::Level::DEBUG, "Expanding octree upwards");
		// Create new, larger root node
		Vector3f center_dist = object->center() - root_node_->center();
		Vector3f new_center = 
				root_node_->center() + (root_node_->width() / 2) * 
				Vector3f((0 <= center_dist(0)) - (center_dist(0) < 0),
				         (0 <= center_dist(1)) - (center_dist(1) < 0),
				         (0 <= center_dist(2)) - (center_dist(2) < 0));
		float new_width = root_node_->width() * 2;
		if(new_width > max_width_) {
			LOG(logging::Level::WARNING, "Tried expanding upwards more than maximum width");
			auto host = object->moveTo(nullptr);
			mutex_.unlock();
			return host;
		}

		Node* new_root_node = new Node(new_center, new_width, this);

		// Calculate locations of new children
		array<Vector3f, 8> children_centers = new_root_node->childrenCenters_();

		for(uint8_t i = 0; i < 8; i++) {
			if((children_centers[i] - root_node_->center()).norm() < 
			   root_node_->width() / 4) {
				// Child is old root
				new_root_node->children_[i] = root_node_;
			} else {
				// Child is new
				new_root_node->children_[i] = new Node(children_centers[i], 
				                                       root_node_->width(),
				                                       this,
				                                       new_root_node);
			}
		}
		root_node_ = new_root_node;
		mutex_.unlock();

		// Now that root note is updated, try adding to it again.
		return add(object);;
	}
}

} // namespace octree