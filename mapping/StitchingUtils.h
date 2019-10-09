#ifndef FILE_STITCHING_UTILS_H
#define FILE_STITCHING_UTILS_H

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

/**
 * Mainly this contains some routines to render lines
 */

inline Vector2f roundVec2Nearest(Vector2f in) {
	return Vector2f(round(in[0]), round(in[1]));
}

inline float getPosOnLine(Vector2f pa, Vector2f pb, Vector2f pr) {
	pa = roundVec2Nearest(pa);
	pb = roundVec2Nearest(pb);
	pr = roundVec2Nearest(pr);
	float t = (pr - pa).dot(pb - pa) / (pb - pa).squaredNorm();
	if(isnan(t)) {
		t = 1; // TODO: You sure about this?
	}
	return t;
}

template <typename T>
T interpolatePerspectific(T fa, T fb, float wa, float wb, float t) {
	T f = (((1.0f - t) * fa) / wa + (t * fb) / wb) / 
	      ((1.0f - t) / wa + t / wb);
	return f;
}

template <typename T>
void bresenham(Vector2f pix0, Vector2f pix1, T f) {
	float deltax = pix1[0] - pix0[0];
	float deltay = pix1[1] - pix0[1];
	if(abs(deltax) < abs(deltay)) {
		int incx = deltax > 0 ? 1 : -1;
		float deltaerr = abs(deltax / deltay); //assuming line is vertical
		float error = deltaerr - 0.5f;
		int x = round(pix0[0]);
		for(int y = round(pix0[1]);
		    deltay > 0 ? y <= round(pix1[1]) : y >= round(pix1[1]);
		    deltay > 0 ? y++ : y--) {
			float t = getPosOnLine(pix0, pix1, Vector2f(x, y));
			f(x, y, t);
			error = error + deltaerr;
			if(error >= 0.5) {
				x = x + incx;
				error = error - 1.0;
			}
		}
	} else {
		int incy = deltay > 0 ? 1 : -1;
		float deltaerr = abs(deltay / deltax); //assuming line is not vertical
		float error = deltaerr - 0.5f;
		int y = round(pix0[1]);
		for(int x = round(pix0[0]);
		    deltax > 0 ? x <= round(pix1[0]) : x >= round(pix1[0]);
		    deltax > 0 ? x++ : x--) {
			float t = getPosOnLine(pix0, pix1, Vector2f(x, y));
			f(x, y, t);
			error = error + deltaerr;
			if(error >= 0.5) {
				y = y + incy;
				error = error - 1.0;
			}
		}
	}
}

/**
 * @brief isOnSameSide
 * Method which tests
 * @param line0
 * @param line1
 * @param point0
 * @param point1
 * @return
 */
inline bool isOnSameSide(Vector2f line0, Vector2f line1, 
                         Vector2f point0, Vector2f point1) {
	Vector2f line = line1 - line0;
	Vector2f one = point0 - line0;
	Vector2f other = point1 - line0;
	float cross_one = line[0] * one[1] - one[0] * line[1];
	float cross_other = line[0] * other[1] - other[0] * line[1];

	return sgn(cross_one) == sgn(cross_other);
}

inline bool isEqual(Vector2f point1, Vector2f point2) {
	return (point1[0] == point2[0]) && (point1[1] == point2[1]);
}

#endif // FILE_STITCHING_UTILS_H
