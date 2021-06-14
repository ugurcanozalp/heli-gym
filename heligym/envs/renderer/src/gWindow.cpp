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
    glfwWindowHint(GLFW_SAMPLES, 4);

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

    // To close off the v-sync, set swap interwal of glfw.
    glfwSwapInterval(0);

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
    glEnable(GL_MULTISAMPLE);

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

    // Create Uniform Buffer Object to reduce need memory in GPU

    // for UBObject block
    glGenBuffers(1, &this->UBO);
    glBindBuffer(GL_UNIFORM_BUFFER, this->UBO);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(glm::mat4) + sizeof(glm::vec3), NULL, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    // define the range of the buffer that links to a uniform binding point which is 0 for UBObjects block
    glBindBufferRange(GL_UNIFORM_BUFFER, 0, this->UBO, 0, sizeof(glm::mat4) + sizeof(glm::vec3)); 

    // for LightBlock block
    glGenBuffers(1, &this->LIGHT);
    glBindBuffer(GL_UNIFORM_BUFFER, this->LIGHT);
    glBufferData(GL_UNIFORM_BUFFER, 4 * sizeof(glm::vec3), NULL, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    // define the range of the buffer that links to a uniform binding point which is 1 for LightBlock block
    glBindBufferRange(GL_UNIFORM_BUFFER, 1, this->LIGHT, 0, 4 * sizeof(glm::vec3));

    // use vec4 for handling std140 format for uniform buffer objects
    this->light_position = glm::vec4(0.0f, 1500.0f, 0.0f, 0.0f);
    glm::vec4 light_ambient = glm::vec4(0.8f, 0.8f, 0.8f, 0.0f);
    glm::vec4 light_diffuse = glm::vec4(255.0 / 255.0f, 241.0 / 255.0f, 242.0 / 255.0f, 0.0f);
    glm::vec4 light_specular = glm::vec4(1.0f, 1.0f, 1.0f, 0.0f);

    glBindBuffer(GL_UNIFORM_BUFFER, this->LIGHT);
    glBufferSubData(GL_UNIFORM_BUFFER,                     0, sizeof(glm::vec4), glm::value_ptr(this->light_position));
    glBufferSubData(GL_UNIFORM_BUFFER,     sizeof(glm::vec4), sizeof(glm::vec4), glm::value_ptr(light_ambient));
    glBufferSubData(GL_UNIFORM_BUFFER, 2 * sizeof(glm::vec4), sizeof(glm::vec4), glm::value_ptr(light_diffuse));
    glBufferSubData(GL_UNIFORM_BUFFER, 3 * sizeof(glm::vec4), sizeof(glm::vec4), glm::value_ptr(light_specular));
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    // for FogBlock block
    glGenBuffers(1, &this->FOG);
    glBindBuffer(GL_UNIFORM_BUFFER, this->FOG);
    glBufferData(GL_UNIFORM_BUFFER, 2 * sizeof(glm::vec4), NULL, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    // define the range of the buffer that links to a uniform binding point which is 2 for FogBlock block
    glBindBufferRange(GL_UNIFORM_BUFFER, 2, this->FOG, 0, 2 * sizeof(glm::vec4));

    glm::vec4 fog_color = glm::vec4(0.74f, 0.35f, 0.51f, 0.4f);
    glm::vec4 density_grad = glm::vec4(0.002f, 5.0f, 0.0, 0.0); // density & grad in first 2 elements, other for padding
   
    glBindBuffer(GL_UNIFORM_BUFFER, this->FOG);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::vec4), glm::value_ptr(fog_color));
    glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::vec4), sizeof(glm::vec4), glm::value_ptr(density_grad));
    
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
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

        // Set project_view as Uniform Buffer Objects 
        glBindBuffer(GL_UNIFORM_BUFFER, this->UBO);
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), glm::value_ptr(this->projection_view));
        glBindBuffer(GL_UNIFORM_BUFFER, 0);

        // Set camera position as Uniform Buffer Objects
        glBindBuffer(GL_UNIFORM_BUFFER, this->UBO);
        glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::vec3), glm::value_ptr(this->camera->Position));
        glBindBuffer(GL_UNIFORM_BUFFER, 0);

        // Draw objects.
        this->draw();     

        // Render gui.
        this->renderGUI();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        // Swap buffer to render the context
        glfwSwapBuffers(this->window);

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

    for (int s = 0; s < this->v_guiTextSection.size(); s++)
    {
        // Set window position and size.
        ImGui::SetNextWindowPos(this->v_guiTextSection[s]->position);
        ImGui::SetNextWindowSize(this->v_guiTextSection[s]->size);

        // Render texts.
        std::vector<guiText> v_guiText = *this->v_guiTextSection[s]->textVector;
        
        ImGui::Begin(this->v_guiTextSection[s]->title.c_str());
        
        for (int i = 0; i < v_guiText.size(); i++)
        {
            ImGui::Text(v_guiText[i].str, *v_guiText[i].val);
        }
        
        ImGui::End();
    }

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
		this->permanent_drawables[i]->draw();
	}

	for (int i = 0; i < this->instantaneous_drawables.size(); i++)
	{
		this->instantaneous_drawables[i]->draw();
	}
	this->instantaneous_drawables.clear();
}


int Window::create_guiText(const char* title, ImVec2 position, ImVec2 size)
{
    guiTextSection* temp = new guiTextSection();
    temp->title = title;
    temp->textVector = new std::vector<guiText>();
    temp->position = position;
    temp->size = size;
    this->v_guiTextSection.push_back(temp);
    
    return this->v_guiTextSection.size() - 1;
}


void Window::add_item_to_guiText(int v_guiText_ind, const char* str, float* val)
{
    guiText* temp = new guiText((char*) str, val);
    this->v_guiTextSection[v_guiText_ind]->textVector->push_back(*temp);
    delete temp;
}


void Window::set_guiText(int v_guiText_ind, float* val)
{
    std::vector<guiText>* temp = this->v_guiTextSection[v_guiText_ind]->textVector;
    for (int i = 0; i < temp->size(); i++)
    {
        guiText* tgT = &temp->at(i);
        tgT->val = &val[i];
    }
}

