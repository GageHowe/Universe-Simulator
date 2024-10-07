#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 ourColor;
out vec3 Normal;
out vec3 FragPos;

uniform mat4 model;
uniform mat4 camMatrix;

void main()
{
	FragPos = vec3(model * vec4(aPos, 1.0));
	gl_Position = camMatrix * vec4(FragPos, 1.0);
	ourColor = aColor;
	Normal = normalize(mat3(transpose(inverse(model))) * aPos);
}