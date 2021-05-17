#version 330 core
out vec4 FragColor;

in vec2 TexCoords;
in float d;

uniform sampler2D texture_diffuse;
uniform float opacity;

uniform vec4 fogColor = vec4(0.24f, 0.35f, 0.51f, 0.4f);
const float fogDensity = 0.002;
const float fogGrad = 5;

void main()
{   
    float visibility = 1.0f - exp2(-pow((d  * fogDensity), fogGrad));  
    FragColor = mix(texture(texture_diffuse, TexCoords) * opacity, fogColor, visibility);
}