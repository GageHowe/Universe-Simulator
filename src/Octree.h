// Deprecated

//#pragma once
//
//#include <array>
//#include <memory>
//#include <glm/glm.hpp>
//
//class Body {
//public:
//    double mass;
//    glm::vec3 position;
//    glm::vec3 velocity;
//
//    Body(double m, const glm::vec3& pos, const glm::vec3& vel);
//};
//
//class OctreeNode {
//public:
//    glm::vec3 center;
//    double size;
//    std::array<std::unique_ptr<OctreeNode>, 8> children;
//    Body* body;
//    double total_mass;
//    glm::vec3 center_of_mass;
//
//    OctreeNode(const glm::vec3& c, double s);
//
//    void insert(Body* b);
//    glm::vec3 calculateForce(Body* body, double theta) const;
//
//private:
//    void split();
//    void insertBody(Body* b);
//    int getOctant(const glm::vec3& pos) const;
//    void updateMass(Body* b);
//};
//
//class BarnesHutOctree {
//public:
//    BarnesHutOctree();
//    void buildTree(const std::vector<Body>& bodies);
//    glm::vec3 calculateForce(Body* body, double theta) const;
//
//private:
//    std::unique_ptr<OctreeNode> root;
//};