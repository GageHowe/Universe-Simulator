#pragma once
#include <GL/glew.h>
#include <string>
#include <vector>
#include <glm/glm.hpp>

class Program
{
public:
	
	Program();
	~Program();
	void SetShadersFileName(char *vFileName, char *sFileName);
	void CheckShaderCompileStatus(GLuint shader);
	void Init();
	std::string ReadShader(const char *name);
	void SendVaryingData(std::vector<float> &posBuff, std::vector<float> &norBuff, std::vector<float> &texBuff);
	void SendUniformData(int a, const char* name);
	void SendUniformData(float a, const char* name);
	void SendUniformData(glm::vec3 input, const char* name);
	void SendUniformData(glm::mat4 &mat, const char* name);
	void Bind();
	void Unbind();
	GLint GetPID() { return programID; };


private:
	GLint programID;
	char *vertexShaderFileName, *fragmentShaderFileName;
};

