#version 330 core

layout (location = 0) in vec3 aPos;

uniform mat4 camMatrix;
uniform mat4 model;
uniform vec3 uniformColor;  // New uniform for color

out vec3 color;

void main()
{
	gl_Position = camMatrix * model * vec4(aPos, 1.0);
	color = uniformColor;  // Use the uniform color
}