#ifndef GWINDOW_H
#define GWINDOW_H

#include "camera.h"
#include "model.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <iostream>

#include <chrono>
#include <thread>

// Create basic structure for ImGui text. With this structure
// text gui has only one values for a text. 
struct guiText
{
    char* str;
    float* val;
    guiText() { str = 0; val = 0; }
    guiText(char* _str, float* _val) { str = _str; val = _val; }
};

// Class for GLFW-Window.
class Window
{
private:
    // Window properties and its variables.
    unsigned int SCR_WIDTH;
    unsigned int SCR_HEIGHT;
    std::chrono::duration<long, std::nano> deltaTime;
    std::chrono::steady_clock::time_point lastFrame = std::chrono::steady_clock::now();
    float lastX = 0.0f;
    float xoffset = 0.0f;
    float lastY = 0.0f;
    float yoffset = 0.0f;
    bool firstMouse = true;
    unsigned int UBO; // uniform buffer objects for UBObjects
    unsigned int LIGHT; // uniform buffer objects for LightBlocks
    unsigned int FOG; // uniform buffer objects for FogBlocks
    unsigned int depthMapFBO; // depth frame buffer for shadow
    unsigned int depthMap; // depth texture
    glm::vec4 light_position;


    // Drawable objects vectors. It stores object's pointer to call when they 
    // need to draw. 
    std::vector<Model*> permanent_drawables;
	std::vector<Model*> instantaneous_drawables;

    // Drawing of window which called by render function.
    void draw();

    // Sleep the window for sync the FPS of window when the FPS is higher
    // than dynamic system FPS (which calculated in Python side).
    void preciseSleep(double seconds);

    // Projection view for vertex-shaders for each shader.
    glm::mat4 projection_view = glm::mat4(1.0f);

public:
    // Base window if GLFW.
    GLFWwindow* window;

    // Camera class. 
    Camera* camera;

    // FPS variables and dt which is in nanoseconds.
    float FPS = (float)1e-7;
    float FPS_limit = 100.0; 
    std::chrono::nanoseconds dt{static_cast<long int>( 1000000000.0f/this->FPS_limit)};

    // Observation text vector to print multiple messages which
    // based on guiText structure.
    std::vector<guiText> guiOBS;

    // Basic Window constructor.
    Window() {};

    // Window constructor with Width, Height and Title parameters.
    Window(const unsigned int SCR_WIDTH,
           const unsigned int SCR_HEIGHT,
           const char* title);
    
    // Render the Window.
    void render();

    // Render the Dear ImGui.
    void renderGUI();

    // Setting Observation text for Dear ImGui.
    void set_guiOBS(float* val);

    // Adding model as permanent drawable objects of the Window.
    // It drawing the model until the window closed.
    void add_permanent_drawables(Model* drawable);

    // Adding model as instantaneous drawable objects of the Window.
    // It drawing the model only at that render time.
	void add_instantaneous_drawables(Model* drawable);

    // Callbacks for OpenGL.

    // When window size changed, GLFW call this callback function.
    void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    
    // When mouse moves, GLFW call this call function.
    void mouse_callback(GLFWwindow* window, double xpos, double ypos);

    // When mouse scroll changed (wheel rotate etc.), GLFW call this function.
    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    // Adding str and val to related guiText vector such as guiOBS.
    void add_item_to_guiText(std::vector<guiText>* guiText, const char* str, float* val);

    // GLFW need static callback functions. To handle it, some capsulation should
    // be done. This function makes it.
    static void static_framebuffer_size_callback(GLFWwindow* window, int width, int height);

    // GLFW need static callback functions. To handle it, some capsulation should
    // be done. This function makes it.
    static void static_mouse_callback(GLFWwindow* window, double xpos, double ypos);
    
    // GLFW need static callback functions. To handle it, some capsulation should
    // be done. This function makes it.
    static void static_scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
        
};
#endif