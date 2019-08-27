#include "renderableModel.h"
#include <iostream>

const std::string unlit_frag =
#include "shader/unlit.frag"
;

const std::string unlit_vert =
#include "shader/unlit.vert"
;

using namespace gfx;
using namespace std;
using namespace Eigen;

std::weak_ptr<gfx::GLSLProgram> RenderVerySimpleModel::s_flatProgram;

RenderVerySimpleModel::RenderVerySimpleModel()
{

    if(RenderVerySimpleModel::s_flatProgram.use_count() == 0){
        unlitProgram = std::shared_ptr<GLSLProgram>(new GLSLProgram());
        unlitProgram->compileShader( unlit_frag,
                                 gfx::GLSLShader::GLSLShaderType::FRAGMENT,
                                 "unlit.frag" );
        unlitProgram->compileShader( unlit_vert,
                                 gfx::GLSLShader::GLSLShaderType::VERTEX,
                                 "unlit.vert" );
        unlitProgram->link();
        RenderVerySimpleModel::s_flatProgram = unlitProgram;



    }else{
        unlitProgram = s_flatProgram.lock();
    }






}

RenderVerySimpleModel::~RenderVerySimpleModel()
{
    if(VAO!=0){
        cout << "TODO: delete all these stupid buffers" << endl;
    }
}

void RenderVerySimpleModel::setMesh(std::vector<Vector4f> &vertices,
                                    std::vector<unsigned int> indices)
{

    unlitProgram->use();
    glGenVertexArrays(1,&VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1,&vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER,
                 vertices.size()*sizeof(Vector4f),
                 &vertices[0],GL_STATIC_DRAW);

    glVertexAttribPointer(
        0,                                // attribute
        4,                                // size
        GL_FLOAT,                         // type
        GL_FALSE,                         // normalized?
        sizeof(Vector4f),                                // stride (0 should work as well)
        (void*)0                          // array buffer offset
    );
    glEnableVertexAttribArray(0);


    glGenBuffers(1,&elementBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
                 elementBuffer);

    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sizeof(int)*indices.size(),
                 &indices[0],GL_STATIC_DRAW);
    count = indices.size();

}

void RenderVerySimpleModel::render(Eigen::Matrix4f &camProj)
{
    unlitProgram->use();
    glBindVertexArray(VAO);
    Matrix4f projCamModel = camProj*pose;
    glUniformMatrix4fv(0,1,false,(GLfloat*)&projCamModel);
    glUniform4fv(1,1,(GLfloat*)&color[0]);

    gfx::GLUtils::checkForOpenGLError(
                "[RenderVerySimpleModel::render] Right before rendering "
                "some very simple geometry.");

    glDrawElements(primitiveType,
                   count,
                   GL_UNSIGNED_INT,
                   (void*)0);
    gfx::GLUtils::checkForOpenGLError(
                "[RenderVerySimpleModel::render] Right at rendering "
                "some very simple geometry.");




}

CameraFrustrumRenderableModel::CameraFrustrumRenderableModel(Eigen::Vector4f color,
                                                             Eigen::Vector4f intrinsics,
                                                             Eigen::Vector2f resolution,
                                                             float nearClippingPlane, float farClippingPlane)
{

    float fx=intrinsics[0];
    float fy=intrinsics[1];
    float cx=intrinsics[2];
    float cy=intrinsics[3];

    float rx=resolution[0];
    float ry=resolution[1];
    Vector4f p[4] = {
        Vector4f(cx/fx,cy/fy,1,0),
        Vector4f(-(rx-cx)/fx,cy/fy,1,0),
        Vector4f(cx/fx,-(ry-cy)/fy,1,0),
        Vector4f(-(rx-cx)/fx,-(ry-cy)/fy,1,0)
    };

    Vector4f homo(0,0,0,1);
    vector<Vector4f> vertices = {
        p[0]*nearClippingPlane+homo,
        p[1]*nearClippingPlane+homo,
        p[2]*nearClippingPlane+homo,
        p[3]*nearClippingPlane+homo,
        p[0]*farClippingPlane+homo,
        p[1]*farClippingPlane+homo,
        p[2]*farClippingPlane+homo,
        p[3]*farClippingPlane+homo
    };
    vector<unsigned int> indices = {
        0,1,2,3,0,2,3,1,
        0,4,1,5,2,6,3,7,
        4,5,6,7,4,6,7,5
    };

    setMesh(vertices,indices);


}

CameraFrustrumRenderableModel::~CameraFrustrumRenderableModel()
{

}


WireSphereModel::WireSphereModel(Eigen::Vector4f color, Eigen::Vector4f pos, float radius) {
    int vertexCount = 100;
    vector<Vector4f> vertices(vertexCount*3);
    vector<unsigned int> indices(vertexCount*3*2);

    for(int i=0;i<vertexCount;i++){
        float s = sin(M_PI*2.0*float(i)/float(vertexCount));
        float c = cos(M_PI*2.0*float(i)/float(vertexCount));
        vertices[i + vertexCount*0] = Vector4f(s,c,0,1);
        vertices[i + vertexCount*1] = Vector4f(s,0,c,1);
        vertices[i + vertexCount*2] = Vector4f(0,s,c,1);

        indices[(i + vertexCount * 0)*2 + 0] = vertexCount*0 + i;
        indices[(i + vertexCount * 0)*2 + 1] = vertexCount*0 + i + 1;
        indices[(i + vertexCount * 1)*2 + 0] = vertexCount*1 + i;
        indices[(i + vertexCount * 1)*2 + 1] = vertexCount*1 + i + 1;
        indices[(i + vertexCount * 2)*2 + 0] = vertexCount*2 + i;
        indices[(i + vertexCount * 2)*2 + 1] = vertexCount*2 + i + 1;
        if( i == vertexCount-1){
            indices[(i + vertexCount * 0)*2 + 1] = vertexCount*0 + 0 + 0;
            indices[(i + vertexCount * 1)*2 + 1] = vertexCount*1 + 0 + 0;
            indices[(i + vertexCount * 2)*2 + 1] = vertexCount*2 + 0 + 0;
        }
    }
    setMesh(vertices,indices);
    pose = Matrix4f::Identity();
}

WireSphereModel::~WireSphereModel() {

}

void WireSphereModel::updatePose() {
    pose = Matrix4f::Identity();
    pose.block<3,1>(0,3) = pos;
    pose(0,0) = scale[0];
    pose(1,1) = scale[1];
    pose(2,2) = scale[2];
}

void WireSphereModel::setPosition(Eigen::Vector3f position) {
    pos = position;
    updatePose();
}

void WireSphereModel::setRadius(float radius) {
    scale[0] = scale[1] = scale[2] = radius;
    updatePose();
}
