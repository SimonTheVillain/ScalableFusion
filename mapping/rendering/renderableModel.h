#ifndef FILE_RENDERABLE_MODEL_H
#define FILE_RENDERABLE_MODEL_H

#include <Eigen/Eigen>
#include <shader.h>
#include <memory>


//TODO: rename this!!!
class RenderVerySimpleModel{

private:
    GLuint vertexBuffer = 0;
    GLuint elementBuffer = 0;
    int count;

    GLuint VAO = 0;
    static std::weak_ptr<gfx::GLSLProgram> s_flatProgram;
    std::shared_ptr<gfx::GLSLProgram> unlitProgram;


public:
    struct Vertex{
        Eigen::Vector4f p;
        Eigen::Vector4f c;

        Vertex(Eigen::Vector4f pos,Eigen::Vector4f color){
            p=pos;
            c=color;
        }
        ~Vertex(){}
    };

    RenderVerySimpleModel();
    ~RenderVerySimpleModel();


    GLuint primitiveType = GL_LINES;
    void setMesh(std::vector<Eigen::Vector4f> &vertices, std::vector<unsigned int> indices);


    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    Eigen::Vector4f color = Eigen::Vector4f(1,0,0,1);
    void render(Eigen::Matrix4f &camProj);
};



class CameraFrustrumRenderableModel : public RenderVerySimpleModel{
private:
    //RenderVerySimpleModel model;
public:
    CameraFrustrumRenderableModel(Eigen::Vector4f color, Eigen::Vector4f intrinsics, Eigen::Vector2f resolution,
                                  float nearClippingPlane, float farClippingPlane);


    ~CameraFrustrumRenderableModel();
};


class WireSphereModel : public RenderVerySimpleModel{
//TODO: implement this!
private:

    Eigen::Vector3f color;
    Eigen::Vector3f pos;
    Eigen::Vector3f scale;

    void updatePose();
public:
    WireSphereModel(Eigen::Vector4f color, Eigen::Vector4f pos,float radius);
    ~WireSphereModel();

    void setPosition(Eigen::Vector3f position);
    void setRadius(float radius);




};

#endif
