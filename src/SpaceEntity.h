// SpaceEntity.h
#pragma once
#include <glm/glm.hpp>
#include <Jolt/Jolt.h>

class SpaceEntity {
public:
    SpaceEntity(double mass, const glm::vec3& position, const glm::vec3& velocity);
    virtual ~SpaceEntity() = default;

    virtual void update(float deltaTime) = 0;
    virtual void render() = 0;

    glm::vec3 getPosition() const;
    void applyForce(const glm::vec3& force);

protected:
    double mass;
    glm::vec3 position;
    glm::vec3 velocity;
    JPH::BodyID bodyID;
};