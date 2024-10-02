#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include<glm/common.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <filesystem>

#include "physics.hpp"
#include "shaderClass.h"
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "Texture.h"
#include "Camera.h"
// #include "Octree.h" // body is ambiguous

std::atomic<bool> running(true);
GLFWwindow* window;

const unsigned int width = 1280;
const unsigned int height = 720;

glm::vec3 cameraOffset = glm::vec3(0.0f, 2.0f, -5.0f);
float cameraLerpFactor = 0.1f; // how quickly the camera follows the cube (0.0 to 1.0)


// Vertices coordinates for a cube
GLfloat vertices[] =
{ //     COORDINATES     /        COLORS          /   TexCoord  //
	// Front face
	-0.5f, -0.5f,  0.5f,     0.83f, 0.70f, 0.44f,   0.0f, 0.0f, // Bottom-left
	 0.5f, -0.5f,  0.5f,     0.83f, 0.70f, 0.44f,   1.0f, 0.0f, // Bottom-right
	 0.5f,  0.5f,  0.5f,     0.83f, 0.70f, 0.44f,   1.0f, 1.0f, // Top-right
	-0.5f,  0.5f,  0.5f,     0.83f, 0.70f, 0.44f,   0.0f, 1.0f, // Top-left
	// Back face
	-0.5f, -0.5f, -0.5f,     0.83f, 0.70f, 0.44f,   1.0f, 0.0f, // Bottom-left
	 0.5f, -0.5f, -0.5f,     0.83f, 0.70f, 0.44f,   0.0f, 0.0f, // Bottom-right
	 0.5f,  0.5f, -0.5f,     0.83f, 0.70f, 0.44f,   0.0f, 1.0f, // Top-right
	-0.5f,  0.5f, -0.5f,     0.83f, 0.70f, 0.44f,   1.0f, 1.0f  // Top-left
};

// Indices for vertices order
GLuint indices[] =
{
	// Front face
	0, 1, 2,
	2, 3, 0,
	// Right face
	1, 5, 6,
	6, 2, 1,
	// Back face
	7, 6, 5,
	5, 4, 7,
	// Left face
	4, 0, 3,
	3, 7, 4,
	// Bottom face
	4, 5, 1,
	1, 0, 4,
	// Top face
	3, 2, 6,
	6, 7, 3
};

void checkShaderCompileStatus(GLuint shader) {
	GLint status;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE) {
		GLint logLength;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
		GLchar* buffer = new GLchar[logLength];
		GLsizei bufferSize;
		glGetShaderInfoLog(shader, logLength, &bufferSize, buffer);
		std::cout << "unsuccessful" << std::endl;
		std::cout << buffer << std::endl;
		delete[] buffer;

		return;
	}
	else {
		std::cout << "successful" << std::endl;
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

    window = glfwCreateWindow(width, height, "JopenGL", nullptr, nullptr);
    if (window == nullptr) { std::cout << "Failed to create GLFW window" << std::endl; glfwTerminate(); return -1; }

    glfwMakeContextCurrent(window);

    if (!gladLoadGL()) { std::cout << "Failed to initialize GLAD" << std::endl; return -1; }

	Shader shaderProgram("assets/default.vert", "assets/default.frag");

	VAO VAO1; VAO1.Bind(); // Generates Vertex Array Object and binds it
	VBO VBO1(vertices, sizeof(vertices)); // Generates Vertex Buffer Object and links it to vertices
	EBO EBO1(indices, sizeof(indices)); // Generates Element Buffer Object and links it to indices

	// Links VBO attributes such as coordinates and colors to VAO
	VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 8 * sizeof(float), (void*)0);
	VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	VAO1.LinkAttrib(VBO1, 2, 2, GL_FLOAT, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	VAO1.Unbind();
	VBO1.Unbind();
	EBO1.Unbind();

	// Gets ID of uniform called "scale"
	GLuint uniID = glGetUniformLocation(shaderProgram.ID, "scale");

	std::string parentDir = (filesystem::current_path().filesystem::path::parent_path()).string();
	std::string texPath = "/Resources/YoutubeOpenGL 7 - Going 3D/";

	Texture brickTex((parentDir + texPath + "assets/bricks.png").c_str(), GL_TEXTURE_2D, GL_TEXTURE0, GL_RGBA, GL_UNSIGNED_BYTE);
	brickTex.texUnit(shaderProgram, "tex0", 0);

	double prevTime = glfwGetTime();

	glEnable(GL_DEPTH_TEST);

	Camera camera(width, height, glm::vec3(0.0f, 0.0f, 2.0f));

    glViewport(0, 0, width, height);
    glClearColor(0.07f, 0.13f, 0.17f, 1.0f);

	// global physics stuff
    RegisterDefaultAllocator();
	Trace = TraceImpl;
	JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)
	Factory::sInstance = new Factory();
	RegisterTypes();
	TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
	JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);
	const uint cMaxBodies = 1024;
	const uint cNumBodyMutexes = 0;
	const uint cMaxBodyPairs = 1024;
	const uint cMaxContactConstraints = 1024;
	BPLayerInterfaceImpl broad_phase_layer_interface;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
	ObjectLayerPairFilterImpl object_vs_object_layer_filter;
	PhysicsSystem physics_system;
	physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_vs_object_layer_filter);
	MyBodyActivationListener body_activation_listener;
	physics_system.SetBodyActivationListener(&body_activation_listener);
	MyContactListener contact_listener;
	physics_system.SetContactListener(&contact_listener);
	BodyInterface &body_interface = physics_system.GetBodyInterface();
	BoxShapeSettings floor_shape_settings(Vec3(100.0f, 1.0f, 100.0f));
	floor_shape_settings.SetEmbedded(); // A ref counted object on the stack (base class RefTarget) should be marked as such to prevent it from being freed when its reference count goes to 0.
	ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();
	ShapeRefC floor_shape = floor_shape_result.Get(); // We don't expect an error here, but you can check floor_shape_result for HasError() / GetError()
	BodyCreationSettings floor_settings(floor_shape, RVec3(0.0_r, -1.0_r, 0.0_r), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
	Body *floor = body_interface.CreateBody(floor_settings); // Note that if we run out of bodies this can return nullptr
	body_interface.AddBody(floor->GetID(), EActivation::DontActivate);
	BodyCreationSettings sphere_settings(new SphereShape(0.5f), RVec3(0.0_r, 2.0_r, 0.0_r), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
	BodyID sphere_id = body_interface.CreateAndAddBody(sphere_settings, EActivation::Activate);
	body_interface.SetLinearVelocity(sphere_id, Vec3(0.0f, 10.0f, 0.0f));
	body_interface.SetAngularVelocity(sphere_id, Vec3(1.0f, 1.0f, 1.0f));
	body_interface.AddTorque(sphere_id, Vec3(40.0f, 0.0f, 0.0f));
	const float cDeltaTime = 1.0f / 60.0f;
	physics_system.OptimizeBroadPhase();

	    // START THREADS

		glfwMakeContextCurrent(window);

	    // MAIN LOOP
	while (!glfwWindowShouldClose(window)) {
	    std::cout << "Time elapsed: " << glfwGetTime() - prevTime << std::endl;
	    Quat q = body_interface.GetRotation(sphere_id);
	    float a[4] = { q.GetX(), q.GetY(), q.GetZ(), q.GetW() };
	    std::cout << "sphere rotation in quaternion: " << a[0] << " " << a[1] << " " << a[2] << " " << a[3] << std::endl;
	    glm::quat glmQuat = glm::make_quat(a);
	    glm::mat4 rotationMatrix = glm::mat4_cast(glmQuat); // Convert GLM quaternion to a rotation matrix
	    RVec3 physicsPosition = body_interface.GetCenterOfMassPosition(sphere_id);
	    std::cout << "physics position: " << physicsPosition << std::endl;
	    glm::vec3 cubePosition = glm::vec3(physicsPosition.GetX(), physicsPosition.GetY(), physicsPosition.GetZ());
	    glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), cubePosition);
	    glm::mat4 modelMatrix = translationMatrix * rotationMatrix;

	    // Update camera position and orientation
	    glm::vec3 rotatedOffset = glm::vec3(rotationMatrix * glm::vec4(cameraOffset, 1.0f));
	    glm::vec3 targetCameraPosition = cubePosition + rotatedOffset;
	    camera.Position = glm::mix(camera.Position, targetCameraPosition, cameraLerpFactor);

	    // Calculate camera orientation based on cube's rotation
	    glm::vec3 cameraForward = -glm::normalize(rotatedOffset);
	    glm::vec3 cameraUp = glm::vec3(rotationMatrix * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f));

	    camera.Orientation = cameraForward;
	    camera.Up = cameraUp;

	    physics_system.Update(cDeltaTime, 1, &temp_allocator, &job_system);

	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	    shaderProgram.Activate();

	    // Use the updated Matrix function
	    camera.Matrix(80.0f, 0.1f, 500.0f, shaderProgram, "camMatrix");

	    GLuint modelLoc = glGetUniformLocation(shaderProgram.ID, "model");
	    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));

	    glUniform1f(uniID, 0.5f);
	    brickTex.Bind();
	    VAO1.Bind();
	    glDrawElements(GL_TRIANGLES, sizeof(indices) / sizeof(int), GL_UNSIGNED_INT, 0);
	    glfwSwapBuffers(window);
	    glfwPollEvents();
	}

	// cleanup
	VAO1.Delete();
	VBO1.Delete();
	EBO1.Delete();
	brickTex.Delete();
	shaderProgram.Delete();

    running = false;
    glfwDestroyWindow(window); glfwTerminate();
    return 0;
}