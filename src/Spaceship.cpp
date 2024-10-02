#include "Spaceship.h"

void Spaceship::handleInput(GLFWwindow* window, float deltaTime) {
    glm::vec3 force(0);
    float forceStrength = 1000.0f; // Adjust as needed

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        force += orientation * forceStrength;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        force -= orientation * forceStrength;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        force -= glm::cross(orientation, glm::vec3(0, 1, 0)) * forceStrength;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        force += glm::cross(orientation, glm::vec3(0, 1, 0)) * forceStrength;

    applyForce(force * deltaTime);

    // Handle mouse input for rotation
    // ... (implement mouse look similar to the camera code)
}