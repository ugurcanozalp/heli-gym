#ifndef MODEL_H
#define MODEL_H

#include <stb/stb_image.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "mesh.h"
//#include <glm/gtx/string_cast.hpp> // for string handling of glm.

unsigned int TextureFromFile(const char *path, const std::string &directory);

class Model 
{
public:
    // Model data 
    std::vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    std::vector<Mesh> meshes;
    std::string directory;
    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 base_model = glm::mat4(1.0f);
    glm::vec3 mainrotor = glm::vec3(0.0f);
    glm::vec3 tailrotor = glm::vec3(0.0f);
    Shader* shader;

    // To have different shader for different objecs, shader should be created 
    // for each model. To handle it, shader should be created.
    void create_shader(std::string vertex_shader_file_path, std::string fragment_shader_file_path);

    // Constructor, expects a filepath to a 3D model.
    Model(std::string const &path, 
          std::string vertex_shader_file_path, 
          std::string fragment_shader_file_path
          );

    // Draws the model, and thus all its meshes
    void draw();
    
    // Translate the model to `translation` points.
    void translate(glm::vec3 translation);

    // Rotate the model to that angle.
    void rotate(float angle, glm::vec3 rotation);

    // Scale the model
    void scale(glm::vec3 scaling);
    
private:
    // Loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    void loadModel(std::string const &path);

    // Processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene);

    // Create mesh for the model.
    Mesh processMesh(aiMesh *mesh, const aiScene *scene);

    // Checks all material textures of a given type and loads the textures if they're not loaded yet.
    // the required info is returned as a Texture struct.
    std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, std::string typeName);
};

#endif
