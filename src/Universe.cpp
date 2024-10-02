
// Universe.cpp
#include "Universe.h"
#include <algorithm>
#include <cmath>

const double G = 6.67430e-11; // gravitational constant

Universe::Universe() : playerShip(nullptr), octreeRoot(nullptr) {}

Universe::~Universe() = default;

void Universe::update(float deltaTime) {
    buildOctree();
    updateGravity();

    for (auto& entity : entities) {
        entity->update(deltaTime);
    }

    // Step the physics simulation
    // gPhysicsSystem->Update(deltaTime, 1, &temp_allocator, &job_system);
}

void Universe::render() {
    for (auto& entity : entities) {
        entity->render();
    }
}

void Universe::addEntity(std::unique_ptr<SpaceEntity> entity) {
    if (dynamic_cast<Spaceship*>(entity.get()) && !playerShip) {
        playerShip = static_cast<Spaceship*>(entity.get());
    }
    entities.push_back(std::move(entity));
}

Spaceship* Universe::getPlayerShip() {
    return playerShip;
}

void Universe::buildOctree() {
    if (entities.empty()) return;

    // Find bounds of all entities
    glm::vec3 minBound = entities[0]->getPosition();
    glm::vec3 maxBound = minBound;
    for (const auto& entity : entities) {
        glm::vec3 pos = entity->getPosition();
        minBound = glm::min(minBound, pos);
        maxBound = glm::max(maxBound, pos);
    }

    glm::vec3 center = (minBound + maxBound) * 0.5f;
    float size = glm::length(maxBound - minBound) * 0.5f;

    octreeRoot = std::make_unique<OctreeNode>(center, size);

    for (auto& entity : entities) {
        insertIntoOctree(octreeRoot.get(), entity.get());
    }
}

void Universe::insertIntoOctree(OctreeNode* node, SpaceEntity* entity) {
    if (node->entities.size() < 8 || node->size < 1.0f) { // Adjust this minimum size as needed
        node->entities.push_back(entity);
    } else {
        if (node->children[0] == nullptr) {
            // Subdivide
            float halfSize = node->size * 0.5f;
            for (int i = 0; i < 8; ++i) {
                glm::vec3 newCenter = node->center;
                newCenter.x += ((i & 1) ? halfSize : -halfSize);
                newCenter.y += ((i & 2) ? halfSize : -halfSize);
                newCenter.z += ((i & 4) ? halfSize : -halfSize);
                node->children[i] = std::make_unique<OctreeNode>(newCenter, halfSize);
            }

            // Redistribute existing entities
            for (SpaceEntity* existingEntity : node->entities) {
                int octant = getOctant(existingEntity->getPosition(), node->center);
                insertIntoOctree(node->children[octant].get(), existingEntity);
            }
            node->entities.clear();
        }

        // Insert the new entity
        int octant = getOctant(entity->getPosition(), node->center);
        insertIntoOctree(node->children[octant].get(), entity);
    }
}

int Universe::getOctant(const glm::vec3& position, const glm::vec3& center) {
    int octant = 0;
    if (position.x >= center.x) octant |= 1;
    if (position.y >= center.y) octant |= 2;
    if (position.z >= center.z) octant |= 4;
    return octant;
}

void Universe::updateGravity() {
    const float theta = 0.5f; // Barnes-Hut opening angle

    for (auto& entity : entities) {
        glm::vec3 totalForce = calculateForce(octreeRoot.get(), entity.get(), theta);
        entity->applyForce(totalForce);
    }
}

glm::vec3 Universe::calculateForce(OctreeNode* node, SpaceEntity* entity, float theta) {
    glm::vec3 force(0.0f);

    if (node->entities.size() == 1 && node->entities[0] == entity) {
        return force; // Skip self-interaction
    }

    glm::vec3 centerOfMass(0.0f);
    float totalMass = 0.0f;

    // Calculate center of mass for this node
    for (SpaceEntity* nodeEntity : node->entities) {
        centerOfMass += nodeEntity->getPosition() * static_cast<float>(nodeEntity->getMass());
        totalMass += static_cast<float>(nodeEntity->getMass());
    }

    if (totalMass > 0) {
        centerOfMass /= totalMass;
    }

    float distance = glm::length(centerOfMass - entity->getPosition());

    if (node->entities.size() == 1 || (node->size / distance < theta)) {
        // Use this node as a single body
        if (distance > 0) {
            glm::vec3 direction = glm::normalize(centerOfMass - entity->getPosition());
            float forceMagnitude = G * entity->getMass() * totalMass / (distance * distance);
            force = direction * forceMagnitude;
        }
    } else {
        // Recursively calculate force for children
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]) {
                force += calculateForce(node->children[i].get(), entity, theta);
            }
        }
    }

    return force;
}