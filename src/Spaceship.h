#pragma once
#include "SpaceObject.h"
#include <GLFW/glfw3.h>

class Spaceship : public SpaceObject {
public:
    Spaceship(double mass, const glm::vec3& position, const glm::vec3& velocity);

    void update(float deltaTime) override;
    void render() override;
    void handleInput(GLFWwindow* window, float deltaTime);

private:
    glm::vec3 orientation;
};