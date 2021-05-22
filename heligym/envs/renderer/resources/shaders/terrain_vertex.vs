#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
layout (location = 3) in vec3 aTangent;
layout (location = 4) in vec3 aBitangent;  


out vec2 TexCoords;
out vec3 Normal;
out vec3 FragPos;
out vec3 ocamPos;
out float d;

uniform mat4 model;

layout (std140) uniform UBObjects
{
    mat4 projection_view;
    vec3 camPos;
};

uniform mat4 inversedTransposedModel;

void main()
{
    TexCoords = aTexCoords;    
    Normal = vec3(inversedTransposedModel * vec4(aNormal, 1.0f));  

    FragPos = vec3(model * vec4(aPos, 1.0f));
    d = distance(FragPos, camPos);
	gl_Position = projection_view * vec4(FragPos, 1.0f);    
}