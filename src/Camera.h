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

    bool firstClick = true; // Prevents the camera from jumping around when first clicking left click

    int width;
    int height;

    float speed = 0.01f;
    float sensitivity = 200.0f;

    float speed = 0.1f;
    float maxSpeed = 1.0f;
    float acceleration = 0.5f;
    float deceleration = 0.8f;
    glm::vec3 velocity = glm::vec3(0.0f);

    // Constructors
    Camera(int width, int height, glm::vec3 position);
    Camera(int width, int height, glm::vec3 position, glm::vec3 up);

    // Updates and exports the camera matrix to the Vertex Shader
    void Matrix(float FOVdeg, float nearPlane, float farPlane, Shader& shader, const char* uniform);
    void Matrix(float FOVdeg, float nearPlane, float farPlane, Shader& shader, const char* uniform, const glm::vec3& new_up);

    // Handles camera inputs
    void Inputs(GLFWwindow* window);

};
#endif