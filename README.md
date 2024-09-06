#README

src/: This directory contains all your source (.cpp) files.
main.cpp: The entry point of your application.
game.cpp: Game logic implementation.
renderer.cpp: Rendering code using GLFW and OpenGL.
physics.cpp: Physics simulation code using Jolt Physics.


include/: This directory contains all your header (.h) files.

game.h: Declarations for game logic.
renderer.h: Declarations for rendering functions.
physics.h: Declarations for physics simulation functions.


lib/: This directory is for third-party libraries that aren't managed by CMake's FetchContent or find_package.

glfw/: GLFW library files (if not using find_package).
glm/: GLM library files (if not using find_package).


assets/: This directory contains all non-code assets for your project.

textures/: Image files for textures.
models/: 3D model files.
shaders/: GLSL shader files.