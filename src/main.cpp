// https://en.wikipedia.org/wiki/Orders_of_magnitude_(mass)
// https://en.wikipedia.org/wiki/Orders_of_magnitude_(length)
// https://www.cs.cmu.edu/afs/cs.cmu.edu/project/scandal/public/papers/dimacs-nbody.pdf

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include <chrono>
#include <thread>
#include <cstdio>
#include <omp.h>
#include <bits/algorithmfwd.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "glm/gtx/string_cast.hpp"
#include <glm/gtc/random.hpp>

#include "shaderClass.h"
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "Camera.h"

class CelestialBody;
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

// This simulation uses megameters and ronnagram as its base units. This achieves a balance of precision and support for large-scale simulations.
const double Mm_to_m = 1e6;  // 1 Mm = 1,000,000 m; the moon is 3.476 Mm wide
const double Rg_to_kg = 1e24; // ronnagrams
const double G_SI = 6.67430e-11; // m^3 kg^-1 s^-2

const double G = G_SI * Rg_to_kg / (Mm_to_m * Mm_to_m * Mm_to_m); // Adjusted gravitational constant for Mm and Rg

float time_step = 60.0f; // Initial time step (in seconds)
bool isPaused = true;

const double objectSize = 1e12f; // determines visible size for bodies in simulation, arbitrary value

float theta = 1.0f; // Barnes-Hut opening angle, controls performance vs accuracy tradeoff

// variables for managing zoom status and bounding planes
constexpr int initialZoom = 2;          int zoomStatus = initialZoom;
constexpr float initialFov = 80.0f;     float fov = initialFov;
constexpr float initialFar = 1000000.0f;   float far = initialFar;
constexpr float initialNear = 1.0f;     float near = initialNear;

// for benchmarking
long int octree_build_time = 0;
long int force_calculation_time = 0;
long int vel_pos_update_time = 0;
long int imgui_render_time = 0;
long int opengl_render_time = 0;

int stepsPerOctreeRebuild = 10;
int stepsPerVisualFrame = 1;

std::vector<CelestialBody> celestialBodies;

// default values for creating new objects in the scene
bool show_create_body_menu = false;
bool show_table = false;
bool show_performance = false;
bool show_help = false;
bool show_data = false;
glm::dvec3 new_body_position(0.0, 0.0, 0.0);
glm::dvec3 new_body_velocity(0.0, 0.0, 0.0);
double new_body_radius = 1.0;
double new_body_mass = 1e7;
glm::vec3 new_body_color(1.0f, 1.0f, 1.0f);

double totalElapsedTime = 0.0; // simulation time
double realTimeElapsed = 0.0;
double frameSimTime = 0.0;

const double MIN_NODE_SIZE = 1e-6; // this stops a stack overflow when objects occupy exactly the same point in space

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
            vertices.push_back(xSegment); // R
            vertices.push_back(ySegment); // G
            vertices.push_back(1.0f - ySegment); // B
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
        double renderScale = 1e-3; // Adjust this factor to make bodies visible
        createSphereMesh(vertices, indices, static_cast<float>(radius * renderScale), 10);

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

    void update(double dt) { // this uses verlet integration
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

            // Add a base case to stop recursion
            if (size > MIN_NODE_SIZE) {
                insertToChild(body);
            } else {
                bodies.push_back(body);
            }

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

void createNewBody(std::vector<CelestialBody>& celestialBodies) {
    celestialBodies.emplace_back(
        new_body_position,
        new_body_velocity,
        new_body_radius,
        new_body_mass,
        new_body_color
    );
}

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
    if (d < 0.1) return;  // Prevent division by zero by ignoring the case where bodies are too close

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

void calculateForcesOmp(std::vector<CelestialBody>& bodies, const OctreeNode* root) {
    #pragma omp parallel for
    for (auto & body : bodies) {
        calculateForce(&body, root);
    }
}

void create_sun() {
    celestialBodies.emplace_back(
        dvec3(0.0, 0.0, 0.0),  // Position in megameters
        dvec3(0.0, 0.0, 0.0),  // Velocity in megameters/sec
        695.7 * std::cbrt(objectSize), // radius, not important
        1988000, // mass in Ronnagrams
        glm::vec3(1.0f, 0.9f, 0.2f) // color
    );
}

void create_sun_2() {
    new_body_position = dvec3(0.0, 0.0, 0.0);
    new_body_velocity = dvec3(0.0, 0.0, 0.0);
    new_body_radius = 695.7 * std::cbrt(objectSize);
    new_body_mass = 1988000;
    new_body_color = glm::vec3(1.0f, 0.9f, 0.2f);
    show_create_body_menu = true;
}

void create_earth() {
    celestialBodies.emplace_back(
        dvec3(149598, 0.0, 0.0),  // Position in megameters
        dvec3(0.0, 0.0, std::sqrt(G * 1988000 / 149598)),  // calculated orbital velocity in megameters/s
        6.37814 * std::cbrt(objectSize), // radius, not important
        5.97,  // Mass in Ronnagrams
        glm::vec3(1.0f, 0.9f, 0.2f) // color
    );
}

void create_earth_2() {
    new_body_position = dvec3(149598, 0.0, 0.0);
    new_body_velocity = dvec3(0.0, 0.0, std::sqrt(G * 1988000 / 149598));
    new_body_radius = 6.37814 * std::cbrt(objectSize);
    new_body_mass = 5.97;
    new_body_color = glm::vec3(0.1f, 0.9f, 1.0f);
    show_create_body_menu = true;
}

void create_galaxy(int num_bodies = 10000) {
    std::random_device rd;
    std::mt19937 gen(rd());

    // Reduced radius and thickness
    std::uniform_real_distribution<> radius_dist(0.0, 5000.0);   // Reduced from 50000 to 5000 Mm
    std::uniform_real_distribution<> angle_dist(0.0, 2.0 * PI);
    std::uniform_real_distribution<> height_dist(-100.0, 100.0); // Reduced thickness from 1000 to 100
    std::uniform_real_distribution<> mass_dist(0.001, 0.01);     // Reduced masses

    for (int i = 0; i < num_bodies; i++) {
        double radius = radius_dist(gen);
        double angle = angle_dist(gen);
        double height = height_dist(gen);

        dvec3 position(
            radius * cos(angle),
            height,
            radius * sin(angle)
        );

        // Significantly reduced orbital velocity
        double orbital_speed = std::sqrt(G * mass_dist(gen) * radius) * 0.01;  // Reduced factor from 0.7 to 0.01

        dvec3 velocity(
            -orbital_speed * sin(angle),
            0.0,
            orbital_speed * cos(angle)
        );

        // Color gradient from blue to white based on radius
        float blue_intensity = static_cast<float>(radius) / 5000.0f;
        glm::vec3 color(
            0.5f + 0.5f * blue_intensity,
            0.5f + 0.5f * blue_intensity,
            1.0f
        );

        celestialBodies.emplace_back(
            position,
            velocity,
            10.0 * std::cbrt(objectSize),
            mass_dist(gen),
            color
        );
    }
}

int main() {
    // OPENGL INITIALIZATION
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x MSAA

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Space Simulation", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glEnable(GL_MULTISAMPLE);
    } else {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    Shader shader("assets/default.vert", "assets/default.frag");
    Shader pointShader("assets/point.vert", "assets/point.frag");

    std::vector<float> pointVertices;
    VAO pointVAO;
    VBO* pointVBO = nullptr;

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.Fonts->AddFontFromFileTTF("assets/Argon.ttf", 14.0f);

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends for ImGui
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // create initial bodies
    // create_sun_2();
    // create_10000();

    int numObjects = celestialBodies.size();

    // Camera setup
    Camera camera(SCR_WIDTH, SCR_HEIGHT, glm::vec3(0.0f, 0.0f, 150.0f));
    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        if (yoffset <= -1) { // zoom out
            if (zoomStatus > 2) {
                zoomStatus = zoomStatus / (2 * -yoffset);
            } else {
                std::cout << "can't zoom out any further" << std::endl;
            }
        } else { // zoom in
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

    int time_since_last_rebuild = 0;

    // MAIN LOOP
    while (!glfwWindowShouldClose(window)) {
        // FRAME COUNTING
        auto bigStart = std::chrono::high_resolution_clock::now();
        float currentFrame = static_cast<float>(glfwGetTime());
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        std::chrono::time_point<std::chrono::system_clock> start;
        std::chrono::time_point<std::chrono::system_clock> finish;
        long int time;

        octree.build(celestialBodies);

        if (!isPaused) {
            frameSimTime = deltaTime * time_step;
            realTimeElapsed += deltaTime;
            // BUILD OCTREE
            if (time_since_last_rebuild >= stepsPerOctreeRebuild) {
                auto start = std::chrono::high_resolution_clock::now();
                octree.build(celestialBodies);
                auto finish = std::chrono::high_resolution_clock::now();
                octree_build_time = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
                time_since_last_rebuild = 0;
                std::cout << "octree build time: " << octree_build_time << std::endl;
            }
            time_since_last_rebuild++;

            // DO PHYSICS
            for (int i = 0; i < stepsPerVisualFrame; i++) { // Subdivide simulation into smaller slices if necessary
                // CALCULATE RELATIVE FORCES FOR ALL BODIES
                auto start = std::chrono::high_resolution_clock::now();
                calculateForcesOmp(celestialBodies, octree.root.get());
                auto finish = std::chrono::high_resolution_clock::now();
                force_calculation_time = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();

                // UPDATE VELOCITY AND POSITION FOR ALL BODIES
                start = std::chrono::high_resolution_clock::now();
                for (auto& body : celestialBodies) {
                    body.update(deltaTime * time_step / stepsPerVisualFrame);
                }
                totalElapsedTime += deltaTime * time_step / stepsPerVisualFrame;
                finish = std::chrono::high_resolution_clock::now();
                vel_pos_update_time = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
            }
        } else {
            frameSimTime = 0;
        }

        numObjects = celestialBodies.size();

        // DO IMGUI THINGS
        start = std::chrono::high_resolution_clock::now();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        glClearColor(0.0f, 0.02f, 0.02f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        {
            ImGui::Begin("Controls");

            // Pause button
            if (ImGui::Button(isPaused ? "Resume" : "Pause")) {
                isPaused = !isPaused;
            }
            ImGui::SameLine();
            ImGui::Text(isPaused ? "Paused" : "Running");

            // Time step control
            ImGui::SliderFloat("Time Step (seconds)", &time_step, 60.0f, 365*3600*24.0f, "%.1f");

            ImGui::SliderInt("Steps per Octree Rebuild", &stepsPerOctreeRebuild, 1, 50);
            ImGui::SliderInt("Subdivisions", &stepsPerVisualFrame, 1, 100);

            ImGui::SliderFloat("Theta", &theta, 0.1f, 2.0f, "%.1f");

            ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

            ImGui::Spacing();
            ImGui::Text("Body Creation");

            if (ImGui::Button("Create New Body")) {
                show_create_body_menu = true;
            }
            if (ImGui::Button("Create Sun")) {
                create_sun_2();
            }
            if (ImGui::Button("Create Earth")) {
                create_earth_2();
            }
            if (ImGui::Button("Create Galaxy")) {
                create_galaxy(); // default = 10000
            }

            if (ImGui::Button("Show Body Editor")) {
                show_table = true;
            }
            if (ImGui::Button("Show Performance Dialog")) {
                show_performance = true;
            }
            if (ImGui::Button("Show Help Menu")) {
                show_help = true;
            }
            if (ImGui::Button("Show Data Menu")) {
                show_data = true;
            }

            ImGui::End();
        }
        if (show_data) {
            ImGui::Begin("Data", &show_data);

            ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::Text("%d Objects", numObjects);

            ImGui::Text("Simulated time per frame: %.3f seconds", frameSimTime);

            // Convert total elapsed time to appropriate units
            if (totalElapsedTime < 60) {
                ImGui::Text("Total simulated time: %.2f seconds", totalElapsedTime);
            } else if (totalElapsedTime < 3600) {
                ImGui::Text("Total simulated time: %.2f minutes", totalElapsedTime / 60.0);
            } else if (totalElapsedTime < 86400) {
                ImGui::Text("Total simulated time: %.2f hours", totalElapsedTime / 3600.0);
            } else {
                ImGui::Text("Total simulated time: %.2f days", totalElapsedTime / 86400.0);
            }

            // Display real time elapsed
            ImGui::Text("Real time elapsed: %.2f seconds", realTimeElapsed);

            // Calculate and display current simulation speed
            double currentSimulationSpeed = (frameSimTime / deltaTime);
            if (currentSimulationSpeed < 1) {
                ImGui::Text("Simulation speed: %.3f simulated seconds per real second%s",
                            currentSimulationSpeed,
                            isPaused ? " (Paused)" : "");
            } else if (currentSimulationSpeed < 60) {
                ImGui::Text("Simulation speed: %.3fx real time%s",
                            currentSimulationSpeed,
                            isPaused ? " (Paused)" : "");
            } else if (currentSimulationSpeed < 3600) {
                ImGui::Text("Simulation speed: %.3f simulated minutes per real second%s",
                            currentSimulationSpeed / 60.0,
                            isPaused ? " (Paused)" : "");
            } else if (currentSimulationSpeed < 86400) {
                ImGui::Text("Simulation speed: %.3f simulated hours per real second%s",
                            currentSimulationSpeed / 3600.0,
                            isPaused ? " (Paused)" : "");
            } else {
                ImGui::Text("Simulation speed: %.3f simulated days per real second%s",
                            currentSimulationSpeed / 86400.0,
                            isPaused ? " (Paused)" : "");
            }

            ImGui::End();
        }
        if (show_performance) {
            ImGui::Begin("Performance", &show_performance);

            ImGui::Text("Building octree took %i microseconds", octree_build_time);
            ImGui::Text("Calculating forces took %i microseconds", force_calculation_time*stepsPerVisualFrame);
            ImGui::Text("Calculating velocities and positions took %i microseconds", vel_pos_update_time*stepsPerVisualFrame);
            ImGui::Text("Rendering ImGui took %i microseconds", imgui_render_time);
            ImGui::Text("Rendering with OpenGL took %i microseconds", opengl_render_time);

            ImGui::End();
        }
        if (show_help) {
            ImGui::Begin("Help", &show_help);
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * ImGui::GetColumnWidth());

            ImGui::Text("Steps per Octree Rebuild: Rebuilding octrees is computationally expensive. Smaller values are more accurate, especially with rapid or chaotic motion.");
            ImGui::Spacing();
            ImGui::Text("Subdivisions: This allows you to subdivide the computation in between frames. Use this to improve accuracy at the cost of performance.");
            ImGui::Spacing();
            ImGui::Text("Theta: This is the Barnes-Hut opening angle and controls performance vs accuracy tradeoff. Smaller values are more accurate but approach O(n^2) territory.");
            ImGui::Spacing();
            ImGui::Text("Simulation speed: This is dynamically computed as the ratio between simulation time and real time. It may look hard-coded due to its unwavering accuracy. It's not.");
            ImGui::End();
        }
        if (show_table)
        {
            ImGui::Begin("Celestial Bodies Editor", &show_table ); // the bool reference makes an x to close out the window and reset the bool

            if (ImGui::BeginTable("Bodies Table", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY))
            {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("ID");
                ImGui::TableSetupColumn("Position (Mm)");
                ImGui::TableSetupColumn("Velocity (Mm/s)");
                ImGui::TableSetupColumn("Force (N)");
                ImGui::TableSetupColumn("Mass (Rg)");
                ImGui::TableSetupColumn("Radius (Mm)");
                ImGui::TableSetupColumn("Color");
                ImGui::TableHeadersRow();

                for (size_t i = 0; i < celestialBodies.size(); i++)
                {
                    auto& body = celestialBodies[i];
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("%zu", i);

                    ImGui::TableNextColumn();
                    {
                        ImGui::PushID((int)(i * 7 + 0)); // this is very hacky but it works for now
                        ImGui::InputDouble("X", &body.position.x, 0.0, 0.0, "%.2f");
                        ImGui::InputDouble("Y", &body.position.y, 0.0, 0.0, "%.2f");
                        ImGui::InputDouble("Z", &body.position.z, 0.0, 0.0, "%.2f");
                        ImGui::PopID();
                    }

                    ImGui::TableNextColumn();
                    {
                        ImGui::PushID((int)(i * 7 + 1));
                        ImGui::InputDouble("X", &body.velocity.x, 0.0, 0.0, "%.2f");
                        ImGui::InputDouble("Y", &body.velocity.y, 0.0, 0.0, "%.2f");
                        ImGui::InputDouble("Z", &body.velocity.z, 0.0, 0.0, "%.2f");
                        ImGui::PopID();
                    }

                    ImGui::TableNextColumn();
                    {
                        ImGui::PushID((int)(i * 7 + 2));
                        ImGui::InputDouble("X", &body.force.x, 0.0, 0.0, "%.2e");
                        ImGui::InputDouble("Y", &body.force.y, 0.0, 0.0, "%.2e");
                        ImGui::InputDouble("Z", &body.force.z, 0.0, 0.0, "%.2e");
                        ImGui::PopID();
                    }

                    ImGui::TableNextColumn();
                    {
                        ImGui::PushID((int)(i * 7 + 3));
                        if (ImGui::InputDouble("##Mass", &body.mass, 0.0, 0.0, "%.3e"))
                        {
                            if (body.mass <= 0) body.mass = std::numeric_limits<double>::min();
                        }
                        ImGui::PopID();
                    }

                    ImGui::TableNextColumn();
                    {
                        ImGui::PushID((int)(i * 7 + 4));
                        if (ImGui::InputDouble("##Radius", &body.radius, 0.0, 0.0, "%.2f"))
                        {
                            if (body.radius <= 0) body.radius = std::numeric_limits<double>::min();
                        }
                        ImGui::PopID();
                    }

                    ImGui::TableNextColumn();
                    {
                        ImGui::PushID((int)(i * 7 + 5));
                        float color[3] = {body.color.r, body.color.g, body.color.b};
                        if (ImGui::ColorEdit3("##Color", color, ImGuiColorEditFlags_NoInputs))
                        {
                            body.color = glm::vec3(color[0], color[1], color[2]);
                        }
                        ImGui::PopID();
                    }
                }
                ImGui::EndTable();
            }

            ImGui::End();
        }
        if (show_create_body_menu) {
            ImGui::Begin("Create A New Body", &show_create_body_menu);

            ImGui::Text("Position (Mm)");
            ImGui::InputDouble("X##pos", &new_body_position.x, 0.1, 1.0);
            ImGui::InputDouble("Y##pos", &new_body_position.y, 0.1, 1.0);
            ImGui::InputDouble("Z##pos", &new_body_position.z, 0.1, 1.0);

            ImGui::Text("Velocity (Mm/s)");
            ImGui::InputDouble("X##vel", &new_body_velocity.x, 0.1, 1.0);
            ImGui::InputDouble("Y##vel", &new_body_velocity.y, 0.1, 1.0);
            ImGui::InputDouble("Z##vel", &new_body_velocity.z, 0.1, 1.0);

            ImGui::InputDouble("Radius (Mm)", &new_body_radius, 0.1, 1.0);
            ImGui::InputDouble("Mass (Rg)", &new_body_mass, 1e-6, 1e-3, "%.3e");

            ImGui::ColorEdit3("Color", &new_body_color[0]);

            ImGui::Spacing();

            if (ImGui::Button("Create Body")) {
                createNewBody(celestialBodies);
                numObjects = celestialBodies.size();
                show_create_body_menu = false;
            }

            ImGui::End();
        }

        finish = std::chrono::high_resolution_clock::now();
        time =  std::chrono::duration_cast<std::chrono::microseconds>(finish-start).count();
        // std::cout << "ImGUI setup took: " << time << " microseconds\n";
        imgui_render_time = time;

        // DO GRAPHICS STUFF
        start = std::chrono::high_resolution_clock::now();
        camera.Inputs(window);
        camera.Matrix(fov, near, far, shader, "camMatrix");
        shader.setVec3("viewPos", camera.Position); // Update view position for specular lighting

        // Update point vertices
        pointVertices.clear();
        for (const auto& body : celestialBodies) {
            pointVertices.push_back(static_cast<float>(body.position.x));
            pointVertices.push_back(static_cast<float>(body.position.y));
            pointVertices.push_back(static_cast<float>(body.position.z));
        }

        // Update or create point VBO
        if (pointVBO == nullptr) {
            pointVBO = new VBO(pointVertices.data(), pointVertices.size() * sizeof(float));
            pointVAO.Bind();
            pointVAO.LinkAttrib(*pointVBO, 0, 3, GL_FLOAT, 3 * sizeof(float), (void*)0);
            pointVAO.Unbind();
            pointVBO->Unbind();
        } else {
            pointVBO->Bind();
            glBufferData(GL_ARRAY_BUFFER, pointVertices.size() * sizeof(float), pointVertices.data(), GL_DYNAMIC_DRAW);
            pointVBO->Unbind();
        }

        // Render points
        pointShader.Activate();
        camera.Matrix(fov, near, far, pointShader, "camMatrix");
        pointVAO.Bind();
        glDrawArrays(GL_POINTS, 0, pointVertices.size() / 3);
        pointVAO.Unbind();

        for (auto& body : celestialBodies) {
            body.draw(shader);
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
        finish = std::chrono::high_resolution_clock::now();
        time = std::chrono::duration_cast<std::chrono::microseconds>(finish-start).count();
        // std::cout << "Rendering stuff took: " << time << " microseconds\n";
        opengl_render_time = time;

        auto bigFinish = std::chrono::high_resolution_clock::now();
        // std::cout << "\nOverall, this frame took: " << std::chrono::duration_cast<std::chrono::microseconds>(bigFinish-bigStart).count() << " microseconds\n\n";
    }

    delete pointVBO;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
