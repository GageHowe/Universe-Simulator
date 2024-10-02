// Deprecated

//#include "Octree.h"
//#include <algorithm>
//#include <cmath>
//
//const double G = 6.67430e-11; // gravitational constant (m^3 kg^-1 s^-2)
//
//Body::Body(double m, const glm::vec3& pos, const glm::vec3& vel)
//    : mass(m), position(pos), velocity(vel) {}
//
//OctreeNode::OctreeNode(const glm::vec3& c, double s)
//    : center(c), size(s), body(nullptr), total_mass(0) {}
//
//void OctreeNode::insert(Body* b) {
//    if (!body && children[0] == nullptr) {
//        body = b;
//        total_mass = b->mass;
//        center_of_mass = b->position;
//        return;
//    }
//
//    if (body) {
//        split();
//        insertBody(body);
//        body = nullptr;
//    }
//
//    insertBody(b);
//}
//
//void OctreeNode::split() {
//    for (int i = 0; i < 8; ++i) {
//        glm::vec3 newCenter = center + glm::vec3(
//            ((i & 1) ? 0.25f : -0.25f) * size,
//            ((i & 2) ? 0.25f : -0.25f) * size,
//            ((i & 4) ? 0.25f : -0.25f) * size
//        );
//        children[i] = std::make_unique<OctreeNode>(newCenter, size * 0.5);
//    }
//}
//
//void OctreeNode::insertBody(Body* b) {
//    int index = getOctant(b->position);
//    children[index]->insert(b);
//    updateMass(b);
//}
//
//int OctreeNode::getOctant(const glm::vec3& pos) const {
//    int index = 0;
//    if (pos.x >= center.x) index |= 1;
//    if (pos.y >= center.y) index |= 2;
//    if (pos.z >= center.z) index |= 4;
//    return index;
//}
//
//void OctreeNode::updateMass(Body* b) {
//    glm::vec3 weightedPos = center_of_mass * static_cast<float>(total_mass) + b->position * static_cast<float>(b->mass);
//    total_mass += b->mass;
//    center_of_mass = weightedPos / static_cast<float>(total_mass);
//}
//
//glm::vec3 OctreeNode::calculateForce(Body* body, double theta) const {
//    glm::vec3 r = center_of_mass - body->position;
//    float distance = glm::length(r);
//
//    if (this->body == body || distance < 1e-10f) return glm::vec3(0);
//
//    if (this->body || (size / distance < theta)) {
//        float f = static_cast<float>(G * body->mass * total_mass / (distance * distance * distance));
//        return r * f;
//    }
//
//    glm::vec3 totalForce(0);
//    for (const auto& child : children) {
//        if (child) {
//            totalForce += child->calculateForce(body, theta);
//        }
//    }
//    return totalForce;
//}
//
//BarnesHutOctree::BarnesHutOctree() : root(nullptr) {}
//
//void BarnesHutOctree::buildTree(const std::vector<Body>& bodies) {
//    if (bodies.empty()) return;
//
//    glm::vec3 min_pos = bodies[0].position, max_pos = bodies[0].position;
//    for (const auto& body : bodies) {
//        min_pos = glm::min(min_pos, body.position);
//        max_pos = glm::max(max_pos, body.position);
//    }
//    glm::vec3 center = (min_pos + max_pos) * 0.5f;
//    float size = glm::max(glm::max(max_pos.x - min_pos.x, max_pos.y - min_pos.y), max_pos.z - min_pos.z) * 1.1f;
//
//    root = std::make_unique<OctreeNode>(center, size);
//    for (auto& body : bodies) {
//        root->insert(&body);
//    }
//}
//
//glm::vec3 BarnesHutOctree::calculateForce(Body* body, double theta) const {
//    if (!root) return glm::vec3(0);
//    return root->calculateForce(body, theta);
//}
//
///* Example usage:
//std::vector<Body> generateRandomBodies(int numBodies, double boxSize) {
//    std::vector<Body> bodies;
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<> dis(-boxSize/2, boxSize/2);
//    std::uniform_real_distribution<> mass_dis(1e10, 1e11);
//
//    for (int i = 0; i < numBodies; ++i) {
//        glm::vec3 position(dis(gen), dis(gen), dis(gen));
//        glm::vec3 velocity(0, 0, 0);  // Start with zero velocity
//        double mass = mass_dis(gen);
//        bodies.emplace_back(mass, position, velocity);
//    }
//
//    return bodies;
//}
//
//int main() {
//    const int numBodies = 1000;
//    const double boxSize = 1e12;  // 1 trillion meters
//    const double theta = 0.5;  // Barnes-Hut opening angle
//
//    // Generate random bodies
//    std::vector<Body> bodies = generateRandomBodies(numBodies, boxSize);
//
//    // Create and build the octree
//    BarnesHutOctree octree;
//    octree.buildTree(bodies);
//
//    // Calculate forces for each body
//    for (auto& body : bodies) {
//        glm::vec3 force = octree.calculateForce(&body, theta);
//
//        // Print the force on this body
//        std::cout << "Force on body at position ("
//                  << body.position.x << ", "
//                  << body.position.y << ", "
//                  << body.position.z << "): ("
//                  << force.x << ", "
//                  << force.y << ", "
//                  << force.z << ")" << std::endl;
//
//        // Here you would typically update the body's velocity and position
//        // body.velocity += force / body.mass * timestep;
//        // body.position += body.velocity * timestep;
//    }
//
//    return 0;
//}
//
//*/
