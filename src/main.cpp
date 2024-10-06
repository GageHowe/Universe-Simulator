// https://www.cs.cmu.edu/afs/cs.cmu.edu/project/scandal/public/papers/dimacs-nbody.pdf

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include <chrono>
#include <thread>
#include <omp.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "glm/gtx/string_cast.hpp"
#include <glm/gtc/random.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "shaderClass.h"
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "Camera.h"

const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

const float G = 0.0000001; //6.67430e-11f;  // The Gravitational constant :)
const float theta = 0.1f;  // Barnes-Hut opening angle, controls performance vs accuracy tradeoff
                            // at 0, there will be no optimization and every particle interacts with every other particle
                            // at values of 1 or greater, Barnes-Hut groups particles much more often, approaching O(n) runtime
                            // this comes at the cost of accuracy

constexpr int initialZoom = 2;          int zoomStatus = initialZoom;
constexpr float initialFov = 80.0f;     float fov = initialFov;
constexpr float initialFar = 5000.0f;   float far = initialFar;
constexpr float initialNear = 1.0f;   float near = initialNear;

#define PI 3.14159265
using dvec3 = glm::dvec3; // double precision vectors

void createSphereMesh(std::vector<float>& vertices, std::vector<unsigned int>& indices, float radius, int segments) {
    vertices.clear();
    indices.clear();

    for (int y = 0; y <= segments; y++) {
        for (int x = 0; x <= segments; x++) {
            float xSegment = (float)x / (float)segments;
            float ySegment = (float)y / (float)segments;
            float xPos = std::cos(xSegment * 2.0f * PI) * std::sin(ySegment * PI) * radius;
            float yPos = std::cos(ySegment * PI) * radius;
            float zPos = std::sin(xSegment * 2.0f * PI) * std::sin(ySegment * PI) * radius;

            vertices.push_back(xPos);
            vertices.push_back(yPos);
            vertices.push_back(zPos);
            vertices.push_back(xSegment); // Use xSegment as r color
            vertices.push_back(ySegment); // Use ySegment as g color
            vertices.push_back(1.0f - ySegment); // Inverse of ySegment as b color
        }
    }

    for (int y = 0; y < segments; y++) {
        for (int x = 0; x < segments; x++) {
            indices.push_back(y * (segments + 1) + x);
            indices.push_back((y + 1) * (segments + 1) + x);
            indices.push_back(y * (segments + 1) + x + 1);

            indices.push_back(y * (segments + 1) + x + 1);
            indices.push_back((y + 1) * (segments + 1) + x);
            indices.push_back((y + 1) * (segments + 1) + x + 1);
        }
    }
}

class CelestialBody {
public:
    dvec3 position;
    dvec3 velocity;
    dvec3 force;
    double radius;
    double mass;
    glm::vec3 color;
    VAO vao;
    VBO* vbo;
    EBO* ebo;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    CelestialBody(const dvec3& pos, const dvec3& vel, double r, double m, const glm::vec3& col)
        : position(pos), velocity(vel), force(0.0, 0.0, 0.0), radius(r), mass(m), color(col), vbo(nullptr), ebo(nullptr) {
        createSphereMesh(vertices, indices, static_cast<float>(radius), 20);

        // std::cout << "Vertices: " << vertices.size() << ", Indices: " << indices.size() << std::endl;

        if (vertices.empty() || indices.empty()) {
            throw std::runtime_error("Failed to create sphere mesh");
        }

        vbo = new VBO(vertices.data(), vertices.size() * sizeof(float));
        ebo = new EBO(indices.data(), indices.size() * sizeof(unsigned int));
        vao.Bind();
        vao.LinkAttrib(*vbo, 0, 3, GL_FLOAT, 6 * sizeof(float), (void*)0);
        vao.LinkAttrib(*vbo, 1, 3, GL_FLOAT, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        vao.Unbind();
        vbo->Unbind();
        ebo->Unbind();
    }

    ~CelestialBody() {
        delete vbo;
        delete ebo;
    }

    // Prevent copying
    CelestialBody(const CelestialBody&) = delete;
    CelestialBody& operator=(const CelestialBody&) = delete;

    CelestialBody(CelestialBody&& other) noexcept
        : position(other.position), velocity(other.velocity), force(other.force),
          radius(other.radius), mass(other.mass), color(other.color),
          vao(std::move(other.vao)), vbo(other.vbo), ebo(other.ebo),
          vertices(std::move(other.vertices)), indices(std::move(other.indices)) {
        other.vbo = nullptr;
        other.ebo = nullptr;
    }

    CelestialBody& operator=(CelestialBody&& other) noexcept {
        if (this != &other) {
            delete vbo;
            delete ebo;
            position = other.position;
            velocity = other.velocity;
            force = other.force;
            radius = other.radius;
            mass = other.mass;
            color = other.color;
            vao = std::move(other.vao);
            vbo = other.vbo;
            ebo = other.ebo;
            vertices = std::move(other.vertices);
            indices = std::move(other.indices);
            other.vbo = nullptr;
            other.ebo = nullptr;
        }
        return *this;
    }

    void draw(Shader& shader) {
        shader.Activate();
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(position));  // Convert to float for rendering
        shader.setMat4("model", model);
        shader.setVec3("color", color);

        vao.Bind();
        ebo->Bind();
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        ebo->Unbind();
        vao.Unbind();
    }

    void update(double dt) { // verlet integration let's goooooooooo
        // First half of position update
        position += velocity * (dt / 2.0);

        // Velocity update
        dvec3 acceleration = force / mass;
        velocity += acceleration * dt;

        // Second half of position update
        position += velocity * (dt / 2.0);

        force = dvec3(0.0, 0.0, 0.0);  // Reset force
    }
};

class OctreeNode {
public:
    dvec3 center;
    double size;
    dvec3 centerOfMass;
    double totalMass;
    std::vector<CelestialBody*> bodies;
    std::unique_ptr<OctreeNode> children[8];

    OctreeNode(const dvec3& center, double size)
        : center(center), size(size), centerOfMass(0.0, 0.0, 0.0), totalMass(0.0) {}

    bool isLeaf() const {
        return children[0] == nullptr;
    }

    int getOctant(const dvec3& position) const {
        int octant = 0;
        if (position.x >= center.x) octant |= 4;
        if (position.y >= center.y) octant |= 2;
        if (position.z >= center.z) octant |= 1;
        return octant;
    }

    void insert(CelestialBody* body) {
        if (isLeaf() && bodies.empty()) {
            bodies.push_back(body);
            centerOfMass = body->position;
            totalMass = body->mass;
        } else {
            if (isLeaf() && bodies.size() == 1) {
                CelestialBody* existingBody = bodies[0];
                bodies.clear();
                subdivide();
                insertToChild(existingBody);
            }
            insertToChild(body);

            // Update center of mass and total mass
            dvec3 weightedPos = centerOfMass * totalMass + body->position * body->mass;
            totalMass += body->mass;
            centerOfMass = weightedPos / totalMass;
        }
    }

private:
    void subdivide() {
        double childSize = size / 2.0;
        for (int i = 0; i < 8; ++i) {
            dvec3 childCenter = center;
            childCenter.x += ((i & 4) ? childSize : -childSize) / 2.0;
            childCenter.y += ((i & 2) ? childSize : -childSize) / 2.0;
            childCenter.z += ((i & 1) ? childSize : -childSize) / 2.0;
            children[i] = std::make_unique<OctreeNode>(childCenter, childSize);
        }
    }

    void insertToChild(CelestialBody* body) {
        int octant = getOctant(body->position);
        children[octant]->insert(body);
    }
};

class Octree {
public:
    std::unique_ptr<OctreeNode> root;

    void build(const std::vector<CelestialBody>& bodies) {

        if (bodies.empty()) return;

        // Find bounding box
        dvec3 min = bodies[0].position, max = bodies[0].position;
        for (const auto& body : bodies) {
            min = glm::min(min, body.position);
            max = glm::max(max, body.position);
        }

        dvec3 center = (min + max) * 0.5;
        double size = glm::length(max - min) * 0.5;

        root = std::make_unique<OctreeNode>(center, size);

        for (const auto& body : bodies) {
            root->insert(const_cast<CelestialBody*>(&body));
        }
    }
};

void calculateForce(CelestialBody* body, const OctreeNode* node) {
    if (node->isLeaf() && node->bodies.empty()) {
        return;
    }

    double d = glm::length(node->centerOfMass - body->position);
    if (d < 2) return;  // Prevent division by zero by having bodies ignore each other when within x units

    if (node->isLeaf() || (node->size / d < theta)) {
        dvec3 direction = glm::normalize(node->centerOfMass - body->position);
        double forceMagnitude = G * body->mass * node->totalMass / (d * d);
        body->force += direction * forceMagnitude;
    } else {
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]) {
                calculateForce(body, node->children[i].get());
            }
        }
    }
}

void calculateForcesNormal(std::vector<CelestialBody>& bodies, const OctreeNode* root) {
    for (auto & body : bodies) {
        calculateForce(&body, root);
    }
}

void calculateForcesThreads(std::vector<CelestialBody>& bodies, const OctreeNode* root) {
    const size_t numThreads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    auto worker = [&](size_t start, size_t end) {
        for (size_t i = start; i < end; ++i) {
            calculateForce(&bodies[i], root);
        }
    };

    size_t chunkSize = bodies.size() / numThreads;
    for (size_t i = 0; i < numThreads - 1; ++i) {
        threads.emplace_back(worker, i * chunkSize, (i + 1) * chunkSize);
    }
    threads.emplace_back(worker, (numThreads - 1) * chunkSize, bodies.size());

    for (auto& thread : threads) {
        thread.join();
    }
}

// holy barnes-hut this is fast
void calculateForcesOmp(std::vector<CelestialBody>& bodies, const OctreeNode* root) {
#pragma omp parallel for
    for (auto & body : bodies) {
        calculateForce(&body, root);
    }
}

glm::vec3 getColorForBody(double mass, double radius) {
    // Example color scheme:
    // - Blue for small, low mass bodies (like planets)
    // - White-yellow for medium bodies
    // - Red for large, high mass bodies (like red giants)

    double massScale = std::log10(mass) / 30.0;  // Assuming masses range from about 1e24 to 1e30
    double radiusScale = std::log10(radius) / 10.0;  // Adjust based on your radius range

    glm::vec3 color;
    if (massScale < 0.3) {
        color = glm::mix(glm::vec3(0.0, 0.0, 1.0), glm::vec3(0.0, 1.0, 1.0), massScale / 0.3);
    } else if (massScale < 0.7) {
        color = glm::mix(glm::vec3(0.0, 1.0, 1.0), glm::vec3(1.0, 1.0, 0.0), (massScale - 0.3) / 0.4);
    } else {
        color = glm::mix(glm::vec3(1.0, 1.0, 0.0), glm::vec3(1.0, 0.0, 0.0), (massScale - 0.7) / 0.3);
    }

    // Adjust brightness based on radius
    color = glm::mix(color, glm::vec3(1.0), radiusScale);

    return color;
}

int main() {

    // OPENGL INITIALIZATION
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Space Simulation", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_DEPTH_TEST);
    Shader shader("assets/default.vert", "assets/default.frag");

    // THIS DOESNT WORK FOR SOME REASON
    // // IMGUI SETUP
    // IMGUI_CHECKVERSION();
    // ImGui::CreateContext();
    // ImGuiIO& io = ImGui::GetIO();
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // IF using Docking Branch
    //
    // // Setup Platform/Renderer backends
    // ImGui_ImplGlfw_InitForOpenGL(window, true);          // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
    // ImGui_ImplOpenGL3_Init();

    // GENERATE BODIES
    std::vector<CelestialBody> celestialBodies;

    double velocityScale = 0.7;
    glm::dvec3 center(0.0, 0.0, 0.0);
    glm::dvec3 up(0.0, 0.0, 1.0);      // rotation axis

    // for random sizes
    std::uniform_real_distribution unif(1e8,3e10);
    std::default_random_engine re;

    for (int i = 0; i < 1000; ++i) {
        glm::dvec3 position = glm::sphericalRand(100.0);
        glm::dvec3 toCenter = center - position;
        glm::dvec3 velocity = glm::cross(up, toCenter);

        // Normalize and scale the velocity
        velocity = glm::normalize(velocity) * glm::length(toCenter) * velocityScale;

        double mass = unif(re);
        celestialBodies.emplace_back(
            position,
            velocity,
            std::cbrt(mass * 0.000000002),
            mass,
            glm::vec3(1.0f, 0.9f, 0.2f)
        );
    }

    // manages camera and zoom
    Camera camera(SCR_WIDTH, SCR_HEIGHT, glm::vec3(0.0f, 0.0f, 150.0f));
    glfwSetScrollCallback(window, [](GLFWwindow* window,double xoffset, double yoffset) {
        if (yoffset <= -1) { // zoom out
            if (zoomStatus > 2) {
                zoomStatus = zoomStatus / (2 * -yoffset);
            } else {
                std::cout << "can't zoom out any further" << std::endl;
            }
        } else {
            // zoom in
       if (zoomStatus <= 2048) {
           zoomStatus = zoomStatus * (2 * yoffset);
       } else {
           std::cout << "can't zoom in any further" << std::endl;
       }
   }
        fov = initialFov * initialZoom / zoomStatus;
        near = initialNear * 8 * zoomStatus;
        far = initialFar / initialZoom * pow(zoomStatus, 1.4); // 1.4 is a temporary value

        std::cout << "Far: " << far << " Fov: " << fov << std::endl;
    });

    // Set up lighting
    glm::vec3 lightPos(10.0f, 10.0f, 10.0f);
    shader.Activate();
    shader.setVec3("lightPos", lightPos);

    float lastFrame = 0.0f;
    Octree octree;

    // MAIN LOOP
    while (!glfwWindowShouldClose(window)) {

        // FRAME COUNTING
        auto bigStart = std::chrono::high_resolution_clock::now();
        float currentFrame = static_cast<float>(glfwGetTime());
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        glClearColor(0.0f, 0.02f, 0.02f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // BUILD OCTREE
        auto start = std::chrono::high_resolution_clock::now();
        octree.build(celestialBodies);
        auto finish = std::chrono::high_resolution_clock::now();
        std::cout << "Building octree took: " << std::chrono::duration_cast<std::chrono::microseconds>(finish-start).count() << " microseconds\n";

        // CALCULATE RELATIVE FORCES FOR ALL BODIES
        start = std::chrono::high_resolution_clock::now();
        calculateForcesOmp(celestialBodies, octree.root.get());
        finish = std::chrono::high_resolution_clock::now();
        std::cout << "Calculating forces took: " << std::chrono::duration_cast<std::chrono::microseconds>(finish-start).count() << " microseconds\n";

        // UPDATE VELOCITY AND POSITION FOR ALL BODIES
        start = std::chrono::high_resolution_clock::now();
        for (auto& body : celestialBodies) {
            body.update(deltaTime);
        }
        finish = std::chrono::high_resolution_clock::now();
        std::cout << "Updating velocity and position took: " << std::chrono::duration_cast<std::chrono::microseconds>(finish-start).count() << " microseconds\n";

        // DO GRAPHICS STUFF
        start = std::chrono::high_resolution_clock::now();
        camera.Inputs(window);
        camera.Matrix(fov, near, far, shader, "camMatrix");
        shader.setVec3("viewPos", camera.Position); // Update view position for specular lighting
        for (auto& body : celestialBodies) {
            body.draw(shader);
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
        finish = std::chrono::high_resolution_clock::now();
        std::cout << "Graphics/OpeGL stuff took: " << std::chrono::duration_cast<std::chrono::microseconds>(finish-start).count() << " microseconds\n";

        auto bigFinish = std::chrono::high_resolution_clock::now();
        std::cout << "\nOverall, this frame took: " << std::chrono::duration_cast<std::chrono::microseconds>(bigFinish-bigStart).count() << " microseconds\n\n";
    }

    glfwTerminate();
    return 0;
}