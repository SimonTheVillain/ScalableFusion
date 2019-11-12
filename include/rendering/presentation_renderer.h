#ifndef FILE_PRESENTATION_RENDERER_H
#define FILE_PRESENTATION_RENDERER_H

#include <memory>

#include <GL/glew.h>
#include <GL/gl.h>

#include <gfx/gpu_tex.h>
#include <gfx/shader.h>

using namespace std;
using namespace Eigen;


class MeshReconstruction;
class ActiveSet;
class GLFWwindow;
class Worker;
class InformationRenderer;
class LowDetailRenderer;
class TextureUpdater;

class PresentationRenderer { //TODO: maybe rename this
public:

	PresentationRenderer(int width = 640, int height = 480);

	~PresentationRenderer();

	void initInContext(int width, int height, MeshReconstruction *map);

	void initInContext(MeshReconstruction* reconstruction);

	void render(ActiveSet *active_set, Matrix4f projection, Matrix4f pose);

	void renderInWindow(MeshReconstruction* reconstruction,
						Matrix4f view, Matrix4f proj,
						bool render_visible_from_cam,
						GLFWwindow *root_context,
						InformationRenderer* information_renderer,
						LowDetailRenderer* low_detail_renderer,
						TextureUpdater* texture_updater);//TODO: check if that texture updater is needed here!

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

	GLuint geom_density_FBO_;//TODO: remove since it probably is unused
	shared_ptr<gfx::GpuTex2D> geom_density_texture_;//TODO: remove since it probably is unused
	GLuint index_FBO_;//TODO: is this really used?
	shared_ptr<gfx::GpuTex2D> index_texture_;//TODO: is this really used?

	GLuint VAO_;
	static weak_ptr<gfx::GLSLProgram> s_rgb_program_;
	shared_ptr<gfx::GLSLProgram> rgb_program_;

	Worker *rendering_active_set_update_worker_;
	GLuint debug_VAO_;
	shared_ptr<gfx::GLSLProgram> debug_program_;

	//MeshReconstruction *map_;
};

#endif // FILE_PRESENTATION_RENDERER_H
