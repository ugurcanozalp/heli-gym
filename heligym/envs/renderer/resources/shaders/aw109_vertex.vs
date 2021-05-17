#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out vec2 TexCoords;
out float d;

uniform mat4 model;
uniform mat4 projection_view;
uniform vec3 camPos;

mat3 rotationMatrixXYZ(vec3 r)
{
float cx = cos(r.x);
float sx = sin(r.x);
float cy = cos(r.y);
float sy = sin(r.y);
float cz = cos(r.z);
float sz = sin(r.z);

return mat3(cy * cz, 	                cy * sz, 	                -sy,
			-cy * sz + sx * sy * cz,	cx * cz + sx * sy * sz,		sx * cy,
			sx * sz + cx * sy * cz,		-sx * cz + cx * sy * sz,	cx * cy);				   
}

uniform vec3 mainrotor;
uniform vec3 tailrotor;

void main()
{   
    TexCoords = aTexCoords;    
    vec3 pos = aPos;

    // main rotor blade rotation
    if ((pos.y > 1.23 && pos.x > -5.2) || (pos.y > 1.1 && pos.x > 3.2))
    {
        pos.x -= 0.19111;
        pos = rotationMatrixXYZ(mainrotor) * pos;
        pos.x += 0.19111;
    }
    
    // tail rotor blade rotation
    if (pos.x < -5.6 && pos.z < -0.4498)
    {
        pos.x += 6.1315;
        pos.y -= 0.662;
        pos = rotationMatrixXYZ(tailrotor) * pos;
        pos.x -= 6.1315;
        pos.y += 0.662;
    }
    vec4 relPos = model * vec4(pos, 1.0f);
    d = distance(relPos, vec4(camPos, 1.0f));
	gl_Position = projection_view * relPos;
}