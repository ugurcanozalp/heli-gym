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

mat3 rotationMatrixXYZ(vec3 r)
{
float cx = cos(r.x);
float sx = sin(r.x);
float cy = cos(r.y);
float sy = sin(r.y);
float cz = cos(r.z);
float sz = sin(r.z);

return mat3(cy * cz                ,    sz          ,  -sy*cz,
			sx * sy - sz * cz * cy ,    cx * cz     ,	sx * cy + sy * sz * cx,
			sx * sz * cy + sy * cx ,    -sx * cz    ,	- sx * sy * sz + cx * cy);				   
}

uniform vec3 mainrotor;
uniform vec3 tailrotor;

void main()
{   
    TexCoords = aTexCoords;  
    Normal = vec3(inversedTransposedModel * vec4(aNormal, 1.0f));  
    ocamPos = camPos;

    vec3 pos = aPos;

    // main rotor blade rotation
    if ((pos.y > 1.24 && pos.x > -4.62) || (pos.y > 1.1 && pos.x > 1.65))
    {
        pos.x -= 0.222994;
        pos = rotationMatrixXYZ(vec3(-1.2086,  0.0, 3.0959)) * pos;
        pos = rotationMatrixXYZ(mainrotor) * pos;
        pos = rotationMatrixXYZ(vec3(1.2086, 0.0, -3.0959)) * pos;
        pos.x += 0.222994;
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
    FragPos = vec3(model * vec4(pos, 1.0f));
    d = distance(FragPos, camPos);
	gl_Position = projection_view * vec4(FragPos, 1.0f);
}