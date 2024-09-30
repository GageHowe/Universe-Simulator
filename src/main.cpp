#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
// #include "Jolt/jolt.h"
#include "physics.hpp"

std::atomic<bool> running(true);
GLFWwindow* window;

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cout << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(800, 800, "OpenGL Multithreaded", nullptr, nullptr);
    if (window == nullptr) { std::cout << "Failed to create GLFW window" << std::endl; glfwTerminate(); return -1; }

    glfwMakeContextCurrent(window);

    if (!gladLoadGL()) { std::cout << "Failed to initialize GLAD" << std::endl; return -1; }

    glViewport(0, 0, 800, 800);
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
	body_interface.SetLinearVelocity(sphere_id, Vec3(0.0f, -5.0f, 0.0f));
	// body_interface.AddTorque(sphere_id, Vec3(1.0f, 1.0f, 1.0f));
	body_interface.SetAngularVelocity(sphere_id, Vec3(1.0f, 1.0f, 1.0f));
	const float cDeltaTime = 1.0f / 60.0f;
	physics_system.OptimizeBroadPhase();

    // START THREADS

	glfwMakeContextCurrent(window);

    // MAIN LOOP
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        // Handle any main thread logic here
    	std::cout << "sphere rotation in quaternion: " << body_interface.GetRotation(sphere_id) << std::endl;
    	physics_system.Update(cDeltaTime, 1, &temp_allocator, &job_system);

    	glClear(GL_COLOR_BUFFER_BIT);

    	glfwSwapBuffers(window);
    }

    // Cleanup
    running = false;
    glfwDestroyWindow(window); glfwTerminate();
    return 0;
}