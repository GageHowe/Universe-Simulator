// Planet.h
#pragma once
#include "SpaceEntity.h"

class Planet : public SpaceEntity {
public:
    Planet(double mass, const glm::vec3& position, const glm::vec3& velocity, float radius);

    void update(float deltaTime) override;
    void render() override;

private:
    float radius;
};