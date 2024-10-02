// Universe.h
#pragma once

#include "SpaceEntity.h"
#include "Planet.h"
#include "Spaceship.h"
#include <vector>
#include <memory>
#include <glm/glm.hpp>

class Universe {
public:
    Universe();
    ~Universe();

    void update(float deltaTime);
    void render();
    void addEntity(std::unique_ptr<SpaceEntity> entity);
    Spaceship* getPlayerShip();

private:
    struct OctreeNode {
        glm::vec3 center;
        float size;
        std::vector<SpaceEntity*> entities;
        std::unique_ptr<OctreeNode> children[8];
        
        OctreeNode(const glm::vec3& c, float s) : center(c), size(s) {}
    };

    std::vector<std::unique_ptr<SpaceEntity>> entities;
    Spaceship* playerShip;
    std::unique_ptr<OctreeNode> octreeRoot;

    void updateGravity();
    void buildOctree();
    void insertIntoOctree(OctreeNode* node, SpaceEntity* entity);
    glm::vec3 calculateForce(OctreeNode* node, SpaceEntity* entity, float theta);
    int getOctant(const glm::vec3& position, const glm::vec3& center);
};
