#ifndef FILE_STITCHING_UTILS_H
#define FILE_STITCHING_UTILS_H

#include <Eigen/Core>
#include <limits>
#include <opencv2/core.hpp>
#include <math.h>


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * Mainly this contains some routines to render lines
 */

inline Eigen::Vector2f roundVec2Nearest(Eigen::Vector2f in){
    return Eigen::Vector2f(std::round(in[0]),std::round(in[1]));
}

inline float getPosOnLine(Eigen::Vector2f pa,Eigen::Vector2f pb,Eigen::Vector2f pr){
    pa=roundVec2Nearest(pa);
    pb=roundVec2Nearest(pb);
    pr=roundVec2Nearest(pr);
    float t=(pr-pa).dot(pb-pa)/(pb-pa).squaredNorm();
    if(std::isnan(t)){
        t=1;
    }
    return t;
}
template <typename T>
T interpolatePerspectific(T fa,T fb,float wa,float wb,float t){
    T f=((1.0f-t)*fa/wa+t*fb/wb)/((1.0f-t)/wa+t/wb);
    return f;
}
template <typename F>
void bresenham(Eigen::Vector2f pix0,Eigen::Vector2f pix1,F f){

    float deltax=pix1[0]-pix0[0];
    float deltay=pix1[1]-pix0[1];
    if(std::abs(deltax) < std::abs(deltay)){
        int incx = deltax>0 ? 1 : -1;
        float deltaerr = std::abs(deltax/deltay); //assuming line is vertical
        float error = deltaerr-0.5f;
        int x=round(pix0[0]);//x=x0
        for(int y=round(pix0[1]);
             deltay>0 ? y<=round(pix1[1]) : y>=round(pix1[1]);
             deltay>0 ? y++ : y--){
            float t=getPosOnLine(pix0,pix1,Eigen::Vector2f(x,y));
            f(x,y,t);
            error=error+deltaerr;
            if(error >= 0.5){
                x=x+incx;
                error=error-1.0;
            }
        }
    }else{
        int incy = deltay>0 ? 1 : -1;
        float deltaerr = std::abs(deltay/deltax); //assuming line is not vertical
        float error = deltaerr-0.5f;
        int y=round(pix0[1]);//y=y0
        for(int x=round(pix0[0]);
             deltax>0 ? x<=round(pix1[0]) : x>=round(pix1[0]);
             deltax>0 ? x++ : x--){
            float t=getPosOnLine(pix0,pix1,Eigen::Vector2f(x,y));
            f(x,y,t);
            error=error+deltaerr;
            if(error >= 0.5){
                y=y+incy;
                error=error-1.0;
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
inline bool isOnSameSide(Eigen::Vector2f line0, Eigen::Vector2f line1,Eigen::Vector2f point0,Eigen::Vector2f point1){
    Eigen::Vector2f line = line1-line0;
    Eigen::Vector2f one = point0-line0;
    Eigen::Vector2f other = point1-line0;
    float crossOne = line[0]*one[1]-one[0]*line[1];
    float crossOther = line[0]*other[1]-other[0]*line[1];

    return sgn(crossOne)==sgn(crossOther);
}

inline bool isEqual(Eigen::Vector2f point1,Eigen::Vector2f point2){
    return (point1[0] == point2[0]) && (point1[1] == point2[1]);
}

#endif
