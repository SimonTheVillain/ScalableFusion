#include <iostream>
#include <memory>
#include <thread>
#include <map>
#include <stdexcept>

#include <boost/program_options.hpp>
//TODO: remove this
//#include <cuda.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glog/logging.h>

#include <logging/logger.h>
#include <video_source/include/source.h>
#include <gfx/camera.h>
#include <mesh_reconstruction.h>
#include <gfx/gpu_tex.h>
#include <cuda/test.h>
#include <scheduler.h>
#include <scheduler_threaded.h>
#include <utils/arcball.h>
#include <rendering/renderable_model.h>
#include <debug_render.h>
#include <gfx/garbage_collector.h>
#include <export/map_exporter.h>
#include <segmentation/incremental_segmentation.h>
#include <utils/perf_meter.h>

//how to measure memory consumption on a shell basis:
//while true; do sleep 0.1; nvidia-smi | grep mapping | grep -oh "[0-9]*MiB" >> mappingMemory.txt ; done
//while true; do sleep 0.1; nvidia-smi | grep ElasticFusion | grep -oh "[0-9]*MiB" >> elfuMemory.txt ; done
// getting klg files from pngs:
//https://github.com/HTLife/png_to_klg

//TODO: Dynamic VBO

//context sharing with cuda:
//https://blog.qt.io/blog/2015/03/03/qt-weekly-28-qt-and-cuda-on-the-jetson-tk1/

using namespace std;
using namespace Eigen;

//qt layouts without designer:
//http://www.bogotobogo.com/Qt/Qt5_LayoutNotUsingDesigner.php
static void error_callback(int error, const char *description) {
	fprintf(stderr, "Error: %s\n", description);
}

//TODO: get coordinates
static float dist_from_ball = 3.0f;
static Vector3f trans;

static Arcball arcball;

static bool capture_left = false;
static bool capture_right = false;
static double xpos_old = 0, ypos_old = 0;
static void cursor_position_callback(GLFWwindow *window, double xpos,
									 double ypos) {
	float dx = static_cast<float>(xpos_old - xpos);
	float dy = static_cast<float>(ypos_old - ypos);
	if(capture_left) {
		float speed = 0.05f;
		Vector4f trans4(-dx * speed, dy * speed, 0, 1);
		trans4 = arcball.getView().inverse() * trans4;
		trans += trans4.block<3, 1>(0, 0);
	}
	if(capture_right) {
		//do the arcball thingy with the right bouse button
		arcball.drag(xpos, ypos);
	}
	xpos_old = xpos;
	ypos_old = ypos;
}

static bool read_out_surface_info = false;
static bool center_camera = false;
void mouse_button_callback(GLFWwindow *window, int button, int action,
						   int mods) {
	if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		capture_left = true;
	}
	if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
		capture_left = false;
	}
	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		capture_right = true;
		arcball.clickBegin(static_cast<float>(xpos_old),
						   static_cast<float>(ypos_old));
	}
	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		capture_right = false;
	}
	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS &&
	   glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
		read_out_surface_info = true;
	}
	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS &&
	   glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
		center_camera = true;
	}
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
	float scale = 0.05f;
	dist_from_ball *= (1.0f + static_cast<float>(yoffset) * scale);
}

static bool render_wireframe   = false;
static bool render_high_detail = true;
static int color_mode   = 0;//5 would be the labelling
static int shading_mode = 0;
static bool disable_rendering = false;
static bool force_destination_geometry = false;
static bool next_single_step = false;
bool paused = false;
void key_callback(GLFWwindow *window, int key, int scancode, int action,
				  int mods) {
	if(key == GLFW_KEY_W && action == GLFW_PRESS) {
		render_wireframe = !render_wireframe;
	}
	if(key == GLFW_KEY_H && action == GLFW_PRESS) {
		render_high_detail = !render_high_detail;
	}
	if(key == GLFW_KEY_C && action == GLFW_PRESS) {
		color_mode++;
		//color mode 0 being the first texture layer
		//color mode 1 being the colorcoded patch ids
		//color mode 2 being the geometry texture
		//color mode 3 being the direct output of normals as color
		//color mode 4 being the direct output of tex references as color
		//color mode 5 shows segmentation of the first segmentation layer

		if(color_mode==6) {
			color_mode=0;
		}
	}
	if(key == GLFW_KEY_S && action == GLFW_PRESS) {
		shading_mode++;
		//shading mode 0 no shading
		//shading mode 1 flat shading (normals derived per triangle)
		//shading mode 2 phong shading (requires normals)
		//shading mode 3 put out the texture coordinates
		if(shading_mode == 4) {
			shading_mode = 0;
		}
	}
	if(key == GLFW_KEY_I && action == GLFW_PRESS) {
		disable_rendering = !disable_rendering;
	}
	if(key == GLFW_KEY_D && action == GLFW_PRESS) {
		force_destination_geometry = !force_destination_geometry;
	}
	if(key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
		paused = !paused;
	}
	if(key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
		next_single_step = true;
	}
}

static bool close_request = false;//TODO: message this to the scheduler

namespace po = boost::program_options;
int main(int argc, const char *argv[]) {
	START_LOGGING(logging::Level::WARNING);

	google::InitGoogleLogging(argv[0]);

	string dataset_path =
			"/home/simon/datasets/tum/rgbd_dataset_freiburg3_cabinet";

	string graph_output_file =
			"/home/simon/datasets/output/graph.txt";

	string output_path;// =
	//"/home/simon/datasets/tum/output/";

	string coarse_output_file;// =
	//"/home/simon/datasets/tum/output/coarse.ply";

	string detailed_output_file;// =
	//"/home/simon/datasets/tum/output/fine.ply";

	float replay_speed = 0.1f;
#ifdef VERSION_DEBUG
	replay_speed = 0.1f;
#else
	replay_speed = 1.0f;
#endif // DEBUG
	bool use_dataset_trajectory = false;
	float groundtruth_trajectory_scale = 1.0f;
	bool invert_ground_truth_trajectory = false;

	//seemingly i am getting memory segmentation errors
	//https://stackoverflow.com/questions/9901803/cuda-error-message-unspecified-launch-failure
	bool headless      = false;
	bool auto_quit     = false;
	bool multithreaded = false;
	bool store_result  = true;
	bool hd            = false;
	int  skip_initial_frames = 0;
	float depth_scale = 1.0f;

	//Manage the applications input parameters
	po::options_description desc("Allowed options");
	desc.add_options()
			("help", "produce help message")
			("input,i", po::value<string>(&dataset_path),
			 "Set the TUM like directory to read from.")
			("groundtruth,t", po::bool_switch(&use_dataset_trajectory),
			 "Use the groundtruth trajectory that comes with the dataset")
			("multithreaded,m", po::bool_switch(&multithreaded),
			 "Split the capturing process up into multiple threads")
			("startFrame", po::value<int>(&skip_initial_frames),
			 "Skipping the first frames to start at frame n")
			("HD,h", po::bool_switch(&hd),
			 "Using HD textures")
			("headless,h", po::bool_switch(&headless),
			 "Hiding windows of live reconstruction.")
			("output,o", po::value<string>(&output_path),
			 "Storing the reconstruction in this folder. Full reconstruction with "
			 "coarse and detailled reconstruction + "
			 "textures (not implemented).")
			("coarse", po::value<string>(&coarse_output_file),
			 "File to store the coarse representation in. Preferrably a ply file.")
			("detailed", po::value<string>(&detailed_output_file),
			 "File to store the detailed representation in. Preferrably a ply file.")
			("scaleGroundtruth", po::value<float>(&groundtruth_trajectory_scale),
			 "Scale the trajectory so it might match the scale for the depthmap")
			("invertGroundtruth", po::bool_switch(&invert_ground_truth_trajectory),
			 "You might have inverted all of your coordinates.... if so then i advise you to set this flag.")
			("scaleDepth,s", po::value<float>(&depth_scale),"Scale the input depth so we end up in mm resolution.")
			("autoquit,q", po::bool_switch(&auto_quit),
			 "Close the window and quit everything when finished running through the dataset")
			("singlestep", po::bool_switch(&paused),
			 "Not autorunning but requiring single steps for updating the frame");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	//if the programm runns headless then we automatically let it run headless.
	auto_quit = auto_quit || headless;

	//the translation of the current camera:
	//TODO: handle this differently
	trans[2] = -4;

	//create the context that is later used by the other thread to update the geometry
	GLFWwindow* invisible_window;
	glfwSetErrorCallback(error_callback);
	if(!glfwInit())
		exit(EXIT_FAILURE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_VISIBLE, 0);
	invisible_window = glfwCreateWindow(640, 480, "HOPE U NO VISIBLE", nullptr,
										nullptr);
	if(!invisible_window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glewExperimental = GL_TRUE;
	glewInit();
	glGetError();//just to get rid of that one error glew introduces at initialization
	//from here on we have opengl

	GarbageCollector garbage_collector;

	//create a map object that takes 640 by 480 sized depth images
	shared_ptr<MeshReconstruction> scalable_map =
			make_shared<MeshReconstruction>(invisible_window, &garbage_collector,
											multithreaded, 640, 480);

	video::TuwDataset dataset(dataset_path, true);

	shared_ptr<IncrementalSegmentation> incremental_segmentation =
			make_shared<EdithSegmentation>();

	LowDetailRenderer low_detail_renderer;

	TextureUpdater texture_updater;
	SchedulerBase *scheduler = nullptr;
	if(multithreaded) {
		scheduler = new SchedulerThreaded(scalable_map, &dataset, invisible_window);
	} else {
		scheduler = new SchedulerLinear(scalable_map, &garbage_collector, &dataset,
										invisible_window,
										&low_detail_renderer,
										incremental_segmentation);
	}
	scheduler->pause(paused);

	//create an window with the necessary opengl context
	GLFWwindow *window;
	if(!glfwInit())
		exit(EXIT_FAILURE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_VISIBLE, 1);
	if(headless) {
		glfwWindowHint(GLFW_VISIBLE, 0);
	}
	window = glfwCreateWindow(1280, 800, "SUPERMAPPING", nullptr, invisible_window);

	if(!window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	scalable_map->initInGLRenderingContext();

	PresentationRenderer presentation_renderer;
	InformationRenderer information_renderer;//TODO: setup low detail renderer


	//initialize the debug outputs
	that_one_debug_rendering_thingy = new RenderDebugInfo();

	//TODO: implement proper destructor for these and destroy before cloing opengl context
	CameraFrustrumRenderableModel cam_model(Vector4f(0, 1, 0, 1),//color is red
											Vector4f(535.4f, 539.2f,
													 320.1f, 247.6f),//this is just about right
											Vector2f(640, 480),
											0.1f, 2.0f);
	WireSphereModel wire_sphere_model(Vector4f(0, 1, 0, 1), Vector4f(0, 0, 0, 1), 1);

	//set the callback functions for the different inputs
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetKeyCallback(window, key_callback);

	gfx::GLUtils::checkForOpenGLError("[main] Error creating an opengl context!");

	while(!glfwWindowShouldClose(window) && (dataset.isRunning() || !auto_quit)) {
		scheduler->pause(paused);
		if(next_single_step) {
			next_single_step = false;
			scheduler->nextStep();
		}
		//generate the view matrix
		Matrix4f proj = Camera::projection(static_cast<float>(M_PI) * 0.3f,
										   800.0f / 600.0f);
		Matrix4f translation = Matrix4f::Identity();
		translation.block<3, 1>(0, 3) = trans;
		Matrix4f trans_from_arcball = Matrix4f::Identity();
		trans_from_arcball(2, 3) = -dist_from_ball;
		Matrix4f view = trans_from_arcball * arcball.getView() * translation;

		glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		presentation_renderer.show_wireframe = render_wireframe;
		presentation_renderer.color_mode = color_mode;
		presentation_renderer.shading_mode = shading_mode;


		presentation_renderer.initInContext(scalable_map.get());
		presentation_renderer.initInContext(640, 480, scalable_map.get()); //TODO: is this the right resolution?

		low_detail_renderer.initInGlContext();
		if(!disable_rendering) {
			presentation_renderer.renderInWindow(scalable_map.get(),
												 view, proj,
												 render_high_detail,
												 invisible_window,
												 &information_renderer,
												 &low_detail_renderer,
												 &texture_updater);
		}
		if(read_out_surface_info == true) {
			cout << "Reading out the clicked patch to get further info" << endl;
			int patch_ind;
			int triangle_ind;
			Vector4f clicked_point =
					information_renderer.renderAndExtractInfo(
							scalable_map.get(),
							view, proj, &low_detail_renderer,
							render_high_detail, // render stuff thats visible from camera perspective
							invisible_window, 1280, 800,
							static_cast<int>(xpos_old), static_cast<int>(ypos_old),
							&patch_ind, &triangle_ind);
			if(!isnan(clicked_point[0])) {
				shared_ptr<MeshPatch> patch = scalable_map->getPatchById(patch_ind);
				if(patch != nullptr) {
					wire_sphere_model.setPosition(patch->center());
					wire_sphere_model.setRadius(patch->radius());
				}
			}
			read_out_surface_info = false;
		}

		if(center_camera == true) {
			cout << "Reading out the clicked patch to get further info and center the camera" << endl;
			Vector4f center =
					information_renderer.renderAndExtractInfo(
							scalable_map.get(),
							view, proj,
							&low_detail_renderer,
							render_high_detail, invisible_window, 1280, 800,
							static_cast<int>(xpos_old), static_cast<int>(ypos_old));
			if(!isnan(center[0])) {
				trans = -center.block<3, 1>(0, 0);
			}
			center_camera = false;
		}

		cam_model.pose = scheduler->getLastKnownDepthPose();
		Matrix4f proj_view = proj * view;
		cam_model.render(proj_view);

		//maybe deactive all the other stuff
		that_one_debug_rendering_thingy->force_dst_geom = force_destination_geometry;
		that_one_debug_rendering_thingy->render(proj, view);

		//display everything that got rendered
		glfwSwapBuffers(window);
		garbage_collector.collect();
		//react to any user input
		glfwPollEvents();

		//setting the camera parameters... maybe it is wise to set this only when the window size changes
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		arcball.setFramebufferData(width, height);
	}

	if(!graph_output_file.empty()) {
		//MapExporter::storeDeformationGraph(scaleableMap.get(),graphOutputFile);
		//MapExporter::storeGraph(scaleableMap.get(),graphOutputFile);
	}

	if(!coarse_output_file.empty()) {
		//TODO: make the exporter more of an integral part of the scaleableMap
		//MapExporter::ExportMap(map,"/home/simon/Documents/reconstruction_results/");
		//store the stupid reconstruction somewhere
		MapExporter::storeCoarse(scalable_map.get(), &low_detail_renderer, coarse_output_file);
	}

	if(!detailed_output_file.empty()) {
		//store the stupid reconstruction somewhere
		MapExporter::storeFine(scalable_map.get(), detailed_output_file);
	}

	close_request =  true;

	cout << "[main] DEBUG everything should be deleted" << endl;
	garbage_collector.forceCollect();
	delete scheduler;

	//erase active sets and patches so really nothing should be on the gpu
	scalable_map->erase();

	scalable_map.reset();

	cout << "DEBUG:most resources should be freed here! look at nvidia-smi (but in fact nothing is freed)" << endl;

	cout << "Debug:set brakepoint here, the next key on the opencv window should quit the app" << endl;

	glfwDestroyWindow(window);
	glfwDestroyWindow(invisible_window);
	glfwTerminate();

	cout << "Debug after deletion of at least two of the opengl contexts" << endl;

	//TODO: this and the include at the very top
	void *ptr;
	cudaMalloc(&ptr, 10240);
	cout << "DEBUG now exiting the program but purposefully leaving a tiny leak (cuda-memcheck sanity check)" << endl;
	cudaDeviceReset();//this is necessary for a proper memory leak analysis with cuda-memcheck
	
	STOP_LOGGING();
	return 0;
}
