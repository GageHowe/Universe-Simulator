#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Include your provided implementations
#include "shaderClass.h"
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "Camera.h"

const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;

#define PI 3.14159265

// Function to create a sphere mesh
void createSphereMesh(std::vector<float>& vertices, std::vector<unsigned int>& indices, float radius, int segments) {
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
            indices.push_back((y + 1) * (segments + 1) + x);
            indices.push_back(y * (segments + 1) + x);
            indices.push_back(y * (segments + 1) + x + 1);

            indices.push_back((y + 1) * (segments + 1) + x);
            indices.push_back(y * (segments + 1) + x + 1);
            indices.push_back((y + 1) * (segments + 1) + x + 1);
        }
    }
}

class CelestialBody {
public:
    glm::vec3 position;
    glm::vec3 velocity;
    float radius;
    glm::vec3 color;
    VAO vao;
    VBO* vbo;
    EBO* ebo;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    CelestialBody(const glm::vec3& pos, const glm::vec3& vel, float r, const glm::vec3& col)
        : position(pos), velocity(vel), radius(r), color(col), vbo(nullptr), ebo(nullptr) {
        createSphereMesh(vertices, indices, radius, 20);
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

    // Allow moving
    CelestialBody(CelestialBody&& other) noexcept
        : position(other.position), velocity(other.velocity), radius(other.radius), color(other.color),
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
            radius = other.radius;
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
        model = glm::translate(model, position);
        shader.setMat4("model", model);
        shader.setVec3("color", color);
        vao.Bind();
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        vao.Unbind();
    }
};

int main() {
    // Initialize GLFW
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

    std::vector<CelestialBody> celestialBodies;

    // Create a star
    celestialBodies.emplace_back(glm::vec3(0.0f), glm::vec3(0.0f), 2.0f, glm::vec3(1.0f, 0.9f, 0.2f));

    // Create planets
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-20.0, 20.0);
    std::uniform_real_distribution<> colorDis(0.0, 1.0);

    for (int i = 0; i < 8; ++i) {
        glm::vec3 position(dis(gen), dis(gen), dis(gen));
        glm::vec3 velocity(dis(gen) * 0.1f, dis(gen) * 0.1f, dis(gen) * 0.1f);
        float radius = 0.3f + static_cast<float>(i) * 0.1f;
        glm::vec3 color(colorDis(gen), colorDis(gen), colorDis(gen));
        celestialBodies.emplace_back(position, velocity, radius, color);
    }

    Camera camera(SCR_WIDTH, SCR_HEIGHT, glm::vec3(0.0f, 0.0f, 30.0f));

    float lastFrame = 0.0f;

    while (!glfwWindowShouldClose(window)) {
        float currentFrame = static_cast<float>(glfwGetTime());
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        camera.Inputs(window);
        camera.Matrix(45.0f, 0.1f, 100.0f, shader, "camMatrix");

        for (auto& body : celestialBodies) {
            body.position += body.velocity * deltaTime;
            body.draw(shader);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}