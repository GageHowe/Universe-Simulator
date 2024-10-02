// SpaceEntity.cpp
#include "SpaceEntity.h"

#include <Jolt/Jolt.h> // The Jolt headers don't include Jolt.h. Always include Jolt.h before including any other Jolt header.
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

extern JPH::PhysicsSystem* gPhysicsSystem;

SpaceEntity::SpaceEntity(double mass, const glm::vec3& position, const glm::vec3& velocity)
    : mass(mass), position(position), velocity(velocity) {
    // Create Jolt body
    JPH::BodyCreationSettings bodySettings(new JPH::SphereShape(1.0f), JPH::RVec3(position.x, position.y, position.z), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
    bodySettings.mMass = mass;
    bodySettings.mLinearVelocity = JPH::Vec3(velocity.x, velocity.y, velocity.z);
    bodyID = gPhysicsSystem->GetBodyInterface().CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
}

glm::vec3 SpaceEntity::getPosition() const {
    JPH::Vec3 pos = gPhysicsSystem->GetBodyInterface().GetCenterOfMassPosition(bodyID);
    return glm::vec3(pos.GetX(), pos.GetY(), pos.GetZ());
}

void SpaceEntity::applyForce(const glm::vec3& force) {
    gPhysicsSystem->GetBodyInterface().AddForce(bodyID, JPH::Vec3(force.x, force.y, force.z));
}