#ifndef CAMERA_CLASS_H
#define CAMERA_CLASS_H

#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include<glm/gtx/rotate_vector.hpp>
#include<glm/gtx/vector_angle.hpp>

#include"shaderClass.h"

class Camera
{
public:
	// Stores the main vectors of the camera
	glm::vec3 Position;
	glm::vec3 Orientation = glm::vec3(0.0f, 0.0f, -1.0f);
	glm::vec3 Up = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 BodyRotation = glm::vec3(0.0f);

	// Prevents the camera from jumping around when first clicking left click
	bool firstClick = true;

	// Stores the width and height of the window
	int width;
	int height;

	// Adjust the speed of the camera and it's sensitivity when looking around
	float speed = 5.0;
	float sensitivity = 200.0f;

	// Constructor
	Camera(int width, int height, glm::vec3 position);

	// New constructor - Gage
	Camera(int width, int height, glm::vec3 position, glm::vec3 up);

	// Updates and exports the camera matrix to the Vertex Shader
	void Matrix(float FOVdeg, float nearPlane, float farPlane, Shader& shader, const char* uniform);

	// New constructors for updating rotation with quaternions
	void Matrix(float FOVdeg, float nearPlane, float farPlane, Shader& shader, const char* uniform, glm::vec3& new_up);
	void Matrix(float FOVdeg, float nearPlane, float farPlane, Shader& shader, const char* uniform, const glm::vec3& new_up);


	// Handles camera inputs
	void Inputs(GLFWwindow* window);
};
#endif