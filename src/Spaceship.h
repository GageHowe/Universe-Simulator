// Spaceship.h
#pragma once
#include "SpaceEntity.h"
#include <GLFW/glfw3.h>

class Spaceship : public SpaceEntity {
public:
    Spaceship(double mass, const glm::vec3& position, const glm::vec3& velocity);

    void update(float deltaTime) override;
    void render() override;
    void handleInput(GLFWwindow* window, float deltaTime);

private:
    float acceleration;
    glm::vec3 orientation;
};