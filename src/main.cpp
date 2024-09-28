#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include "Jolt/jolt.h"
#include "physics.hpp"

class GameState {
    std::mutex stateMutex;
public:
    void update() {
        std::lock_guard<std::mutex> lock(stateMutex);
        std::cout << "\n\n";
        // Add your physics update logic here
    }

    void render() {
        std::lock_guard<std::mutex> lock(stateMutex);
        std::cout << ".";
        // Add your OpenGL rendering code here
    }
};

GameState gameState;
std::atomic<bool> running(true);
GLFWwindow* window;

void simulationThread() {
    const std::chrono::duration<double> timeStep(1.0 / 60.0);
    auto previousTime = std::chrono::high_resolution_clock::now();

    while (running) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - previousTime);

        if (elapsedTime >= timeStep) {
            gameState.update();
            previousTime = currentTime;
        } else {
            std::this_thread::sleep_for(timeStep - elapsedTime);
        }
    }
}

void renderThread() {
    glfwMakeContextCurrent(window);

    while (running) {
        glClear(GL_COLOR_BUFFER_BIT);

        gameState.render();
        glfwSwapBuffers(window);
    }
}

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cout << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(800, 800, "OpenGL Multithreaded", NULL, NULL);
    if (window == NULL) { std::cout << "Failed to create GLFW window" << std::endl; glfwTerminate(); return -1; }

    glfwMakeContextCurrent(window);

    if (!gladLoadGL()) { std::cout << "Failed to initialize GLAD" << std::endl; return -1; }

    glViewport(0, 0, 800, 800);
    glClearColor(0.07f, 0.13f, 0.17f, 1.0f);

    // START THREADS
    std::thread simThread(simulationThread);
    std::thread rendThread(renderThread);

    // MAIN LOOP
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        // Handle any main thread logic here
    }

    // Cleanup
    running = false;
    simThread.join(); rendThread.join();
    glfwDestroyWindow(window); glfwTerminate();
    return 0;
}