#ifndef SHADER_H
#define SHADER_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>



class Shader
{
public:
    // Shader variable. 
    unsigned int ID;

    // Constructor generates the shader.
    Shader(const char* vertexPath, const char* fragmentPath)
    {
        // Retrieve the vertex/fragment source code from filePath
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;

        // Ensure ifstream objects can throw exceptions:
        vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        try
        {
            // Open files
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);
            std::stringstream vShaderStream, fShaderStream;
            
            // Read file's buffer contents into streams
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            
            // Close file handlers
            vShaderFile.close();
            fShaderFile.close();
            
            // Convert stream into string
            vertexCode = vShaderStream.str();
            fragmentCode = fShaderStream.str();
        }
        catch (std::ifstream::failure& e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ | " << vertexPath << " " << e.what() << std::endl;
        }

        const char* vShaderCode = vertexCode.c_str();
        const char* fShaderCode = fragmentCode.c_str();
        
        // Compile shaders
        unsigned int vertex, fragment;
        
        // Vertex shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        checkCompileErrors(vertex, "VERTEX");
        
        // Fragment Shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        checkCompileErrors(fragment, "FRAGMENT");
        
        // Shader Program
        this->ID = glCreateProgram();
        glAttachShader(this->ID, vertex);
        glAttachShader(this->ID, fragment);
        glLinkProgram(this->ID);
        checkCompileErrors(this->ID, "PROGRAM");
        
        // Delete the shaders as they're linked into program and no longer necessery
        glDeleteShader(vertex);
        glDeleteShader(fragment);

    }

    // Activating the shader.
    void use() const
    {
        glUseProgram(this->ID);
    }

    // Utility uniform functions
    
    // Setting boolean type uniform by its name.
    void setBool(const std::string& name, bool value) const
    {
        glUniform1i(glGetUniformLocation(this->ID, name.c_str()), (int)value);
    }
    
    // Setting integer type uniform by its name.
    void setInt(const std::string& name, int value) const
    {
        glUniform1i(glGetUniformLocation(this->ID, name.c_str()), value);
    }
    
    // Setting float type uniform by its name.
    void setFloat(const std::string& name, float value) const
    {
        glUniform1f(glGetUniformLocation(this->ID, name.c_str()), value);
    }
    
    // Setting vec2 type uniform by its name.
    void setVec2(const std::string& name, const glm::vec2& value) const
    {
        glUniform2fv(glGetUniformLocation(this->ID, name.c_str()), 1, &value[0]);
    }

    // Setting vec2 type uniform by its name.
    void setVec2(const std::string& name, float x, float y) const
    {
        glUniform2f(glGetUniformLocation(this->ID, name.c_str()), x, y);
    }

    // Setting vec3 type uniform by its name.
    void setVec3(const std::string& name, const glm::vec3& value) const
    {
        glUniform3fv(glGetUniformLocation(this->ID, name.c_str()), 1, &value[0]);
    }

    // Setting vec3 type uniform by its name.
    void setVec3(const std::string& name, float x, float y, float z) const
    {
        glUniform3f(glGetUniformLocation(this->ID, name.c_str()), x, y, z);
    }

    // Setting vec4 type uniform by its name.
    void setVec4(const std::string& name, const glm::vec4& value) const
    {
        glUniform4fv(glGetUniformLocation(this->ID, name.c_str()), 1, &value[0]);
    }

    // Setting vec4 type uniform by its name.
    void setVec4(const std::string& name, float x, float y, float z, float w) const
    {
        glUniform4f(glGetUniformLocation(this->ID, name.c_str()), x, y, z, w);
    }
    
    // Setting mat2 type uniform by its name.
    void setMat2(const std::string& name, const glm::mat2& mat) const
    {
        glUniformMatrix2fv(glGetUniformLocation(this->ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }

    // Setting mat3 type uniform by its name.
    void setMat3(const std::string& name, const glm::mat3& mat) const
    {
        glUniformMatrix3fv(glGetUniformLocation(this->ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    
    // Setting mat4 type uniform by its name.
    void setMat4(const std::string& name, const glm::mat4& mat) const
    {
        glUniformMatrix4fv(glGetUniformLocation(this->ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    
private:
    // Utility function for checking shader compilation/linking errors.
    void checkCompileErrors(GLuint shader, std::string type)
    {
        GLint success;
        GLchar infoLog[1024];
        if (type != "PROGRAM")
        {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success)
            {
                glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
        else
        {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success)
            {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\nThe INFO :: \n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
    }
};
#endif