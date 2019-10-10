#ifndef FILE_MAP_PRESENTATION_RENDERER_H
#define FILE_MAP_PRESENTATION_RENDERER_H

#include <memory>

#include <GL/glew.h>
#include <GL/gl.h>

#include <gfx/gpu_tex.h>
#include <gfx/shader.h>

using namespace std;
using namespace Eigen;

//TODO: rename this to something like presentationRenderer

class MeshReconstruction;
class ActiveSet;
class GLFWwindow;
class Worker;

class MapPresentationRenderer { //TODO: maybe split this up
public:

	MapPresentationRenderer(int width = 640, int height = 480);

	~MapPresentationRenderer();

	void initInContext(int width, int height, MeshReconstruction *map);

	void initInContext();

	void render(ActiveSet *active_set, Matrix4f projection, Matrix4f pose);

	void renderInWindow(Matrix4f view, Matrix4f proj, 
	                    bool render_visible_from_cam,
	                    GLFWwindow *root_context);

	// render an additional wireframe
	bool show_wireframe;

	// all the other render modes.
	bool render_patch_ids;

	int color_mode;

	int shading_mode;

private:

	int width_;
	int height_;

	shared_ptr<gfx::GpuTex2D> debug_texture_;

	GLuint geom_density_FBO_;
	shared_ptr<gfx::GpuTex2D> geom_density_texture_;
	GLuint index_FBO_;
	shared_ptr<gfx::GpuTex2D> index_texture_;

	GLuint VAO_;
	static weak_ptr<gfx::GLSLProgram> s_rgb_program_;
	shared_ptr<gfx::GLSLProgram> rgb_program_;

	Worker *rendering_active_set_update_worker_;
	GLuint debug_VAO_;
	shared_ptr<gfx::GLSLProgram> debug_program_;

	MeshReconstruction *map_;
};

#endif // FILE_GEOMETRY_UPDATE_H
