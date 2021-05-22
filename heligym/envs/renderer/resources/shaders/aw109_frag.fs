#version 330 core
out vec4 FragColor;

in vec2 TexCoords;
in vec3 FragPos;  
in vec3 Normal; 
in vec3 ocamPos;
in float d;

uniform sampler2D texture_diffuse;

uniform float opacity;

struct Light {
    vec4 position;
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
};

layout (std140) uniform LightBlock
{
    Light light;
};

struct Fog 
{
    vec4 color;
    vec4 density_grad;
};

layout (std140) uniform FogBlock
{
    Fog fog;
};


void main()
{   
    // Fog 
    float visibility = 1.0f - exp2(-pow((d  * fog.density_grad[0]), fog.density_grad[1]));  

    // Ambient
    vec3 ambient = vec3(light.ambient) * vec3(light.diffuse);

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(vec3(light.position) - FragPos);  
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * vec3(light.diffuse);

    // Specular
    vec3 viewDir = normalize(ocamPos - FragPos);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(viewDir, halfwayDir), 0.0), 128);
    vec3 specular = vec3(light.specular) * spec * vec3(light.diffuse);  

    vec4 result = vec4(ambient + diffuse + specular, 1.0f) * texture(texture_diffuse, TexCoords) * opacity;

    FragColor = mix(result, fog.color, visibility);
}