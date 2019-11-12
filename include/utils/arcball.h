#ifndef FILE_ARCBALL_H
#define FILE_ARCBALL_H

//https://en.wikibooks.org/wiki/OpenGL_Programming/Modern_OpenGL_Tutorial_Arcball
#include <Eigen/Core>
//angle axis rotation
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class Arcball {
public:

	Arcball()
			: view_(Matrix4f::Identity()),
			  last_x_(0),
			  last_y_(0),
			  width_(800),
			  height_(600) {
	}

	void setFramebufferData(int width, int height) {
		width_  = width;
		height_ = height;
	}

	void clickBegin(float x,float y) {
		last_x_ = x;
		last_y_ = y;
		last_ = getArcballVec_(x, y);
	}

	void drag(float x, float y) {
		Vector3f current = getArcballVec_(x, y);
		float angle = acos(min(1.0f, current.dot(last_)));
		Vector3f axis = current.cross(last_);
		last_ = current;
		Matrix4f m;
		m = Matrix4f::Identity();
		m.block<3, 3>(0, 0) = AngleAxisf(angle, 
		                                 axis.normalized()).toRotationMatrix();
		view_ = m * view_;
	}

	Matrix4f getView() {
		return view_;
	}

private:

	Vector3f getArcballVec_(int x, int y) {
		//normalized point on screen (y is inverted)
		Vector3f P(  1.0 * x / width_  * 2 - 1.0, 
		           -(1.0 * y / height_ * 2 - 1.0), 
		           0);
		float length_2 = P[0] * P[0] + P[1] * P[1];
		if(length_2 < 1.0) {
			P[2] = sqrt(1.0f - length_2);
		} else {
			P = P.normalized();
		}
		length_2 = P[0] * P[0] + P[1] * P[1] + P[2] * P[2];
		return P;
	}

	float last_x_;
	float last_y_;
	int width_;
	int height_;

	Vector3f last_;

	Matrix4f view_;
	
};

#endif // FILE_ARCBALL_H
