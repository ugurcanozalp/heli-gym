#ifndef MESH_H
#define MESH_H

#include "shader.h"

// Vertex structure for store related parameters.
struct Vertex {
    // position
    glm::vec3 Position;
    // normal
    glm::vec3 Normal;
    // texCoords
    glm::vec2 TexCoords;
    // tangent
    glm::vec3 Tangent;
    // bitangent
    glm::vec3 Bitangent;
};

// Texture store structure.
struct Texture {
    unsigned int id;
    std::string type;
    std::string path;
};

class Mesh {
public:
    // Mesh Data
    std::vector<Vertex>       vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture>      textures;
    std::vector<float>        opacity;
    unsigned int VAO;
    std::string opacityname = "opacity";

    // Constructor
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices, std::vector<Texture> textures, std::vector<float> opacity);
    
    // Render the mesh
    void draw(Shader &shader);

private:
    // Render data 
    unsigned int VBO, EBO;

    // Initializes all the buffer objects/arrays
    void setupMesh();
};
#endif