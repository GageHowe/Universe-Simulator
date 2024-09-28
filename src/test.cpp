//#include <thread>
//#include <chrono>
//#include <atomic>
//#include <mutex>
//
//class GameState {
//    // Game state variables
//    std::mutex stateMutex;
//public:
//    void update() {
//        std::lock_guard<std::mutex> lock(stateMutex);
//        // Update physics and game logic here
//    }
//
//    void render() {
//        std::lock_guard<std::mutex> lock(stateMutex);
//        // Render the current game state using OpenGL
//    }
//};
//
//GameState gameState;
//std::atomic<bool> running(true);
//
//void simulationThread() {
//    const std::chrono::duration<double> timeStep(1.0 / 60.0);
//    auto previousTime = std::chrono::high_resolution_clock::now();
//
//    while (running) {
//        auto currentTime = std::chrono::high_resolution_clock::now();
//        auto elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - previousTime);
//
//        if (elapsedTime >= timeStep) {
//            gameState.update();
//            previousTime = currentTime;
//        } else {
//            std::this_thread::sleep_for(timeStep - elapsedTime);
//        }
//    }
//}
//
//void renderThread() {
//    while (running) {
//        gameState.render();
//        // SwapBuffers and handle OpenGL-specific operations
//    }
//}
//
//int main() {
//    // Initialize OpenGL, create window, etc.
//
//    std::thread simThread(simulationThread);
//    std::thread rendThread(renderThread);
//
//    // Main loop (handle input, etc.)
//    while (running) {
//        // Handle input and other non-physics, non-rendering tasks
//    }
//
//    running = false;
//    simThread.join();
//    rendThread.join();
//
//    return 0;
//}