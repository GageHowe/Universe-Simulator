// SpaceObject.h
#pragma once
#include <glm/glm.hpp>
#include <Jolt/Jolt.h>

class SpaceObject {
public:
    SpaceObject(double mass, const glm::vec3& position, const glm::vec3& velocity, bool usePhysics);
    virtual ~SpaceObject();

    virtual void update(float deltaTime);
    virtual void render() = 0;

    glm::vec3 getPosition() const { return position; }
    double getMass() const { return mass; }
    void applyForce(const glm::vec3& force);

protected:
    double mass;
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 acceleration;
    bool usePhysics;
    JPH::BodyID bodyID;

    void syncWithPhysics();
};