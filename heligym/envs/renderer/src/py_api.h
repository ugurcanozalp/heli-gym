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

extern "C" RENDERER_API void create_shader(MainWindow * window, char* path);

extern "C" RENDERER_API MainWindow* create_window(const unsigned int WIDTH,
												  const unsigned int HEIGHT,
												  const char* title);

extern "C" RENDERER_API void close(MainWindow* window);

extern "C" RENDERER_API bool is_close(MainWindow* window);

extern "C" RENDERER_API void render(MainWindow* window);

extern "C" RENDERER_API void terminate();

extern "C" RENDERER_API Model* create_model(char* name);

extern "C" RENDERER_API void add_permanent_to_window(MainWindow* window, Model* model);

extern "C" RENDERER_API void add_instantaneous_to_window(MainWindow* window, Model* model);

extern "C" RENDERER_API void translate_model(Model* model, float x, float y, float z);

extern "C" RENDERER_API void rotate_model(Model* model, float angle, float x, float y, float z);

extern "C" RENDERER_API void scale_model(Model* model, float x, float y, float z);

extern "C" RENDERER_API float get_fps(MainWindow* window);

extern "C" RENDERER_API void set_fps(MainWindow * window, float fps);

extern "C" RENDERER_API Camera* get_camera(MainWindow * window);

extern "C" RENDERER_API void set_camera_pos(Camera* camera, float x, float y, float z);

extern "C" RENDERER_API float* get_camera_pos(Camera * camera);


