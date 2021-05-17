#include "gWindow.h"

Window::Window(const unsigned int SCR_WIDTH,
                        const unsigned int SCR_HEIGHT,
                        const char* title)
{
    // Initialize GLFW.
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);

    // Handle for Apple.
    #ifdef __APPLE__
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    // Create Window.
    this->SCR_WIDTH = SCR_WIDTH;
    this->SCR_HEIGHT = SCR_HEIGHT;
    this->window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, title, NULL, NULL);
    if (this->window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }

    // Set the context.
    glfwMakeContextCurrent(this->window);

    // Initialize GLAD.
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
    }

    // Enable Global OpenGL properties.
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);  
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);  

    // Create camera.
    this->camera = new Camera(glm::vec3(5.415f, 0.2f, 30.0f),
        glm::vec3(0.0f, 1.0f, 0.0f));

    this->lastX = SCR_WIDTH / 2.0f;
    this->lastY = SCR_HEIGHT / 2.0f;

    // Set Input Mode for mouse buttons.
    glfwSetInputMode(this->window, GLFW_STICKY_MOUSE_BUTTONS, GLFW_TRUE);

    // Set callbacks functions to the window and window pointer to OpenGL.
    glfwSetWindowUserPointer(this->window, this);
    glfwSetFramebufferSizeCallback(this->window, Window::static_framebuffer_size_callback);
    glfwSetCursorPosCallback(this->window, Window::static_mouse_callback);
    glfwSetScrollCallback(this->window, Window::static_scroll_callback);

    // Create gui.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui_ImplGlfw_InitForOpenGL(this->window, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    // Add first item of guiOBS as FPS.
    this->add_item_to_guiText(&this->guiOBS, "FPS : %3.f", &this->FPS);

}


void Window::render()
{
    if (!glfwWindowShouldClose(this->window))
    {
        // Clear window for new frame.
        glClearColor(61.0f/255.0, 89.0f/255.0, 129.0f/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                
       
        // Get procection matrix.
        glm::mat4 projection = glm::perspective(glm::radians(this->camera->Zoom), (float)this->SCR_WIDTH / (float)this->SCR_HEIGHT, 0.1f, 10000.0f);

        // Camera/view transformation.
        glm::mat4 view = this->camera->GetViewMatrix();
        this->projection_view = projection * view;
        
        // Draw objects.
        this->draw();     
    
        // Render gui.
        this->renderGUI();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // To close off the v-sync, we used glFlush instead of double buffers.
        glFlush();

        // Poll OpenGL events.
        glfwPollEvents();        

        // If OpenGL FPS is higher than dynamics' FPS (which calculated in Python side)
        // wait some times to sync them.

        // Calculate passed time from last rendered frame.
        this->deltaTime = std::chrono::duration_cast<std::chrono::nanoseconds> (std::chrono::steady_clock::now() - this->lastFrame);

        // Calculate difference of passed time and dt which is calculated by FPS.
        auto diff =  (this->dt - this->deltaTime);

        // If there is difference, wait that much time.
        if (diff.count() > 0)
        {
            this->preciseSleep(diff.count()/1e9);
        } 

        // Calculate FPS from last rendered frame.
        auto dt1 =  std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - this->lastFrame);
        this->FPS = 1.0f / (dt1.count() / 1e9f);

        // Set current time as last frame time.
        this->lastFrame = std::chrono::steady_clock::now();
    }
    else
    {
        // If window should close. Close the Dear ImGui.
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }
}


void Window::preciseSleep(double seconds) 
{
    // Create temporary variables.
    static double estimate = 5e-3;
    static double mean = 5e-3;
    static double m2 = 0;
    static int64_t count = 1;

    // Wait upto seconds.
    while (seconds > estimate) {
        auto start = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        auto end = std::chrono::steady_clock::now();

        double observed = (end - start).count() / 1e9;
        seconds -= observed;

        ++count;
        double delta = observed - mean;
        mean += delta / count;
        m2   += delta * (observed - mean);
        double stddev = sqrt(m2 / (count - 1));
        estimate = mean + stddev;
    }

    // Spin lock handling.
    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start).count() / 1e9 < seconds);
}

void Window::renderGUI()
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Set Observation text location and size.
    ImVec2 info_pos = ImVec2(30.0f, 30.0f);
    ImGui::SetNextWindowPos(info_pos);
    ImGui::SetNextWindowSize(ImVec2(250, 0));

    // Render Observations text.
    ImGui::Begin("Observations!");
    for (int i = 0; i < this->guiOBS.size(); i++)
    {
        ImGui::Text(this->guiOBS[i].str, *this->guiOBS[i].val);
    }
    ImGui::End();

    // Render the gui.
    ImGui::Render();
}


void Window::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}


void Window::mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    // If first frame, set the lastX & lastY as default parameters.
    if (firstMouse)
    {
        lastX = (float)xpos;
        lastY = (float)ypos;
        firstMouse = false;
    }

    // Set offsets.
    this->xoffset = (float)xpos - lastX;
    this->yoffset = lastY - (float)ypos; // reversed since y-coordinates go from bottom to top

    // Set lastX & lastY.
    lastX = (float)xpos;
    lastY = (float)ypos;

    // If left mouse button pressed, change the camera angles.
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        this->camera->ProcessMouseMovement(-this->xoffset, -this->yoffset);
    }       
}


void Window::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    this->camera->ProcessMouseScroll((float)yoffset);
}


void Window::static_framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    Window* temp_window = static_cast<Window*>(glfwGetWindowUserPointer(window));
    temp_window->framebuffer_size_callback(window, width, height);
}


void Window::static_mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    Window* temp_window = static_cast<Window*>(glfwGetWindowUserPointer(window));
    temp_window->mouse_callback(window, xpos, ypos);
}


void Window::static_scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    Window* temp_window = static_cast<Window*>(glfwGetWindowUserPointer(window));
    temp_window->scroll_callback(window, xoffset, yoffset);
}


void Window::add_permanent_drawables(Model* drawable)
{
	this->permanent_drawables.push_back(drawable);
}

void Window::add_instantaneous_drawables(Model* drawable)
{
	this->instantaneous_drawables.push_back(drawable);
}


void Window::draw()
{
	for (int i = 0; i < this->permanent_drawables.size(); i++)
	{
		this->permanent_drawables[i]->draw(this->projection_view, this->camera->Position);
	}

	for (int i = 0; i < this->instantaneous_drawables.size(); i++)
	{
		this->instantaneous_drawables[i]->draw(this->projection_view, this->camera->Position);
	}
	this->instantaneous_drawables.clear();
}


void Window::add_item_to_guiText(std::vector<guiText>* _guiText, const char* str, float* val)
{
    guiText* temp = new guiText((char*) str, val);
    _guiText->push_back(*temp);
    delete temp;
}


void Window::set_guiOBS( float* val)
{
    for (int i = 0; i < this->guiOBS.size(); i++)
    {
        this->guiOBS[i + 1].val = &val[i] ;
    }
}
