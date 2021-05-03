#ifndef MODEL_H
#define MODEL_H

#include <stb/stb_image.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "mesh.h"


using namespace std;

unsigned int TextureFromFile(const char *path, const string &directory, bool gamma = false);

class Model 
{
public:
    // model data 
    vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    vector<Mesh>    meshes;
    string directory;
    bool gammaCorrection;
    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 base_model = glm::mat4(1.0f);

    // constructor, expects a filepath to a 3D model.
    Model(string const &path, bool gamma = false);

    // draws the model, and thus all its meshes
    void draw(Shader &shader);
    
    // translate the model to `translation` points
    void translate(glm::vec3 translation);

    // rotate the model to that angle
    void rotate(float angle, glm::vec3 rotation);

    // scale the model
    void scale(glm::vec3 scaling);

    
private:
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    void loadModel(string const &path);

    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene);

    Mesh processMesh(aiMesh *mesh, const aiScene *scene);

    // checks all material textures of a given type and loads the textures if they're not loaded yet.
    // the required info is returned as a Texture struct.
    vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, string typeName);
};

#endif