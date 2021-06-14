#ifdef _WIN32
	#define BUILDING_DLL
	#ifdef BUILD_RENDERER
		#define RENDERER_API __declspec(dllexport)
	#else
		#define RENDERER_API __declspec(dllimport)
	#endif
#elif linux
	#define RENDERER_API 
#endif

#include "gWindow.h"

// RENDERER API for creating shared libraries to call methods from Python.

// Creating window.
extern "C" RENDERER_API Window* create_window(const unsigned int WIDTH,
										      const unsigned int HEIGHT,
										      const char* title);

// Close window.
extern "C" RENDERER_API void close(Window* window);

// Check the window is closed or not.
extern "C" RENDERER_API bool is_close(Window* window);

// Render the window.
extern "C" RENDERER_API void render(Window* window);

// Terminate the OpenGL.
extern "C" RENDERER_API void terminate();

// Create model with its shaders.
extern "C" RENDERER_API Model* create_model(char* model_path, char* vertex_shader_file_path, char* fragment_shader_file_path);

// Add model as permanent drawables object to window.
extern "C" RENDERER_API void add_permanent_to_window(Window* window, Model* model);

// Add model as instantaneous drawables object to window.
extern "C" RENDERER_API void add_instantaneous_to_window(Window* window, Model* model);

// Translate model to locations. Locations will be in OpenGL coordinates.
extern "C" RENDERER_API void translate_model(Model* model, float x, float y, float z);

// Rotate model to angle. From will be an rotation around arbitrary angle.
extern "C" RENDERER_API void rotate_model(Model* model, float angle, float x, float y, float z);

// Scale model to the ratios for each axis.
extern "C" RENDERER_API void scale_model(Model* model, float x, float y, float z);

// Get FPS from the window.
extern "C" RENDERER_API float get_fps(Window* window);

// Set FPS of the window.
extern "C" RENDERER_API void set_fps(Window* window, float fps);

// Get Camera pointer of the window.
extern "C" RENDERER_API Camera* get_camera(Window* window);

// Set Camera position of each axis.
extern "C" RENDERER_API void set_camera_pos(Camera* camera, float x, float y, float z);

// Get camera position as float array.
extern "C" RENDERER_API float* get_camera_pos(Camera * camera);

// Check whether the window is visible or not.
extern "C" RENDERER_API bool is_visible(Window* window);

// Hide the window.
extern "C" RENDERER_API void hide_window(Window* window);

// Show the window.
extern "C" RENDERER_API void show_window(Window* window);

// Create guiText vector.
extern "C" RENDERER_API int create_guiTextVector(Window * window, const char* title,
													float pos_x, float pos_y,
													float size_x, float size_y);

// Add the str and val to guiText vector.
extern "C" RENDERER_API void add_guiText(Window* window, int v_guiText_ind, int size, char** _str, float* _val);

// Set the guiText of vector.
extern "C" RENDERER_API void set_guiText(Window* window, int v_guiText_ind, float* _val);

// Rotate the model Main Rotor with each angle which are in radians.
extern "C" RENDERER_API void rotate_MR(Model* model, float phi, float theta, float psi);

// Rotate the model Tail Rotor with each angle which are in radians.
extern "C" RENDERER_API void rotate_TR(Model* model, float phi, float theta, float psi);