#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out vec2 TexCoords;
out float d;

uniform mat4 model;
uniform mat4 projection_view;
uniform vec3 camPos;

void main()
{
    TexCoords = aTexCoords;    
    vec4 relPos = model * vec4(aPos, 1.0f);
    d = distance(relPos, vec4(camPos, 1.0f));
	gl_Position = projection_view * relPos;
}