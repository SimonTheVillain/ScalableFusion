#ifndef FILE_ARCBALL_H
#define FILE_ARCBALL_H




//https://en.wikibooks.org/wiki/OpenGL_Programming/Modern_OpenGL_Tutorial_Arcball
#include <Eigen/Core>
//angle axis rotation
#include <Eigen/Geometry>


using namespace Eigen;

class Arcball{
private:
    float m_last_x=0;
    float m_last_y=0;
    float m_cur_x=0;
    float m_cur_y=0;
    int m_width=800;
    int m_height=600;

    Vector3f last;

    Vector3f getArcballVec(int x,int y){
        //normalized point on screen (y is inverted)
        Vector3f P(1.0*x/m_width*2-1.0,-(1.0*y/m_height*2-1.0),0);
        float length_2=P[0]*P[0]+P[1]*P[1];
        if(length_2<1.0){
            P[2]=std::sqrt(1.0f-length_2);
        }else{
            P=P.normalized();
        }
        length_2=P[0]*P[0]+P[1]*P[1]+P[2]*P[2];
        return P;
    }

    Matrix4f view;
public:

    Arcball(){
        view=Matrix4f::Identity();

    }
    ~Arcball(){

    }
    void setFramebufferData(int width,int height){
        m_width=width;
        m_height=height;

    }

    void clickBegin(float x,float y){
        m_last_x=x;
        m_last_y=y;
        last=getArcballVec(x,y);

    }

    void drag(float x,float y){
        Vector3f current=getArcballVec(x,y);
        float angle = std::acos(std::min(1.0f,current.dot(last)));
        Vector3f axis=current.cross(last);
        last=current;
        Matrix4f m;
        m=Matrix4f::Identity();
        m.block<3,3>(0,0)=AngleAxisf(angle,axis.normalized()).toRotationMatrix();
        view=m*view;

    }

    Matrix4f getView(){
        return view;
    }

};
#endif
