// graphics headers
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <iostream>
#include "MatrixStack.h"
#include "Program.h"

// physics headers
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <cstdarg>
#include <thread>
#include <chrono>

#include <AL/al.h>
#include <AL/alc.h>

JPH_SUPPRESS_WARNINGS

// All Jolt symbols are in the JPH namespace
using namespace JPH;

// If you want your code to compile using single or double precision write 0.0_r to get a Real value that compiles to double or float depending on if JPH_DOUBLE_PRECISION is set or not.
using namespace JPH::literals;

// Callback for traces, connect this to your own trace function if you have one
static void TraceImpl(const char *inFMT, ...)
{
	// Format the message
	va_list list;
	va_start(list, inFMT);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), inFMT, list);
	va_end(list);
	// Print to the TTY
	std::cout << buffer << std::endl;
}


#ifdef JPH_ENABLE_ASSERTS
// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{
	// Print to the TTY
	std::cout << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr? inMessage : "") << std::endl;

	// Breakpoint
	return true;
};
#endif // JPH_ENABLE_ASSERTS

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
namespace Layers
{
	static constexpr ObjectLayer NON_MOVING = 0;
	static constexpr ObjectLayer MOVING = 1;
	static constexpr ObjectLayer NUM_LAYERS = 2;
};


/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
	virtual bool					ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
	{
		switch (inObject1)
		{
			case Layers::NON_MOVING:
				return inObject2 == Layers::MOVING; // Non moving only collides with moving
			case Layers::MOVING:
				return true; // Moving collides with everything
			default:
				JPH_ASSERT(false);
			return false;
		}
	}
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers
{
	static constexpr BroadPhaseLayer NON_MOVING(0);
	static constexpr BroadPhaseLayer MOVING(1);
	static constexpr uint NUM_LAYERS(2);
};


// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
	BPLayerInterfaceImpl()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
	}

	virtual uint					GetNumBroadPhaseLayers() const override
	{
		return BroadPhaseLayers::NUM_LAYERS;
	}

	virtual BroadPhaseLayer			GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *			GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		switch ((BroadPhaseLayer::Type)inLayer)
		{
			case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:	return "NON_MOVING";
			case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:		return "MOVING";
			default:													JPH_ASSERT(false); return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	BroadPhaseLayer					mObjectToBroadPhase[Layers::NUM_LAYERS];
};


/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
	virtual bool				ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
	{
		switch (inLayer1)
		{
			case Layers::NON_MOVING:
				return inLayer2 == BroadPhaseLayers::MOVING;
			case Layers::MOVING:
				return true;
			default:
				JPH_ASSERT(false);
			return false;
		}
	}
};

// An example contact listener
class MyContactListener : public ContactListener
{
public:
	// See: ContactListener
	virtual ValidateResult	OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult) override
	{
		std::cout << "Contact validate callback" << std::endl;

		// Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
		return ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	virtual void			OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{ std::cout << "A contact was added" << std::endl; } // play sounds

	virtual void			OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{ std::cout << "A contact was persisted" << std::endl; }

	virtual void			OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
	{ std::cout << "A contact was removed" << std::endl; }
};

// An example activation listener
class MyBodyActivationListener : public BodyActivationListener
{
public:
	virtual void		OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{ std::cout << "A body got activated" << std::endl; }

	virtual void		OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{ std::cout << "A body went to sleep" << std::endl; }
};


#define WINDOW_WIDTH 400
#define WINDOW_HEIGHT 400

char* vertShaderPath = "../assets/shaders/generic_shader.vert";
char* fragShaderPath = "../assets/shaders/generic_shader.frag";

GLFWwindow *window;
double currentXpos, currentYpos;
glm::vec3 eye(0.0f, 0.0f, 8.0f);
glm::vec3 center(0.0f, 0.0f, 0.0f);
glm::vec3 up(0.0f, 1.0f, 0.0f);

Program program;
MatrixStack modelViewProjectionMatrix;

// Draw triangle on screen
void DrawTri(glm::mat4& modelViewProjectionMatrix)
{
	program.SendUniformData(modelViewProjectionMatrix, "mvp");
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}


void Display()
{
	program.Bind();

	modelViewProjectionMatrix.loadIdentity();

	modelViewProjectionMatrix.pushMatrix();

	// Setting the view and Projection matrices
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	modelViewProjectionMatrix.Perspective(glm::radians(60.0f), float(width) / float(height), 0.1f, 100.0f);
	modelViewProjectionMatrix.LookAt(eye, center, up);

	int t = glfwGetTime();
	GLuint timeUnifID = glGetUniformLocation(program.GetPID(), "time");
	glUniform1i(timeUnifID, t);

	DrawTri(modelViewProjectionMatrix.topMatrix());

	modelViewProjectionMatrix.popMatrix();

	program.Unbind();

}

void CreateTri()
{
	// x, y, z, r, g, b, ...
	float triVerts1[] = {
		 -1.0f, -1.0f, 0.0f, 0.8f, 0.2f, 0.4,
	      1.0f, -1.0f, 0.0f, 0.8f, 0.2f, 0.4,
	      0.0f,  1.0f, 0.0f, 0.8f, 0.2f, 0.4,


		  -1.0f, 3.0f, 0.0f, 0.2f, 0.8f, 0.4,
		//  0.0f, 1.0f, 0.0f, 0.2f, 0.8f, 0.4,
		  1.0f,  3.0f, 0.0f, 0.2f, 0.8f, 0.4

	};

	GLuint vertBufferID;
	glGenBuffers(1, &vertBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, vertBufferID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(triVerts1), triVerts1, GL_STATIC_DRAW);
	GLint posID = glGetAttribLocation(program.GetPID(), "position");
	glEnableVertexAttribArray(posID);
	glVertexAttribPointer(posID, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);
	GLint colID = glGetAttribLocation(program.GetPID(), "color");
	glEnableVertexAttribArray(colID);
	glVertexAttribPointer(colID, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));

	GLuint indexBufferID;
	GLuint index1[] = {0,1,2, 3,2,4};
	glGenBuffers(1, &indexBufferID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(index1), index1, GL_STATIC_DRAW);

}

void FrameBufferSizeCallback(GLFWwindow* lWindow, int width, int height)
{
	glViewport(0, 0, width, height);
}

void Init()
{
	glfwInit();
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "TRI", NULL, NULL);
	glfwMakeContextCurrent(window);
	glewExperimental = GL_TRUE;
	glewInit();
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	glfwSetFramebufferSizeCallback(window, FrameBufferSizeCallback);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);

	program.SetShadersFileName(vertShaderPath, fragShaderPath);
	program.Init();

	CreateTri();
}



int main()
{
	Init();
	RegisterDefaultAllocator();											// Register allocation hook. In this example we'll just let Jolt use malloc / free but you can override these if you want (see Memory.h). This needs to be done before any other Jolt function is called.
	Trace = TraceImpl;													// Install trace and assert callbacks
	JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)
	Factory::sInstance = new Factory();									// Create a factory, this class is responsible for creating instances of classes based on their name or hash and is mainly used for deserialization of saved data. It is not directly used in this example but still required.

	RegisterTypes();													// Register all physics types with the factory and install their collision handlers with the CollisionDispatch class. If you have your own custom shape types you probably need to register their handlers with the CollisionDispatch before calling this function. If you implement your own default material (PhysicsMaterial::sDefault) make sure to initialize it before this function or else this function will create one for you.
	TempAllocatorImpl temp_allocator(10 * 1024 * 1024);			// We need a temp allocator for temporary allocations during the physics update. We're pre-allocating 10 MB to avoid having to do allocations during the physics update. B.t.w. 10 MB is way too much for this example but it is a typical value you can use. If you don't want to pre-allocate you can also use TempAllocatorMalloc to fall back to malloc / free.


	JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);	// We need a job system that will execute physics jobs on multiple threads. Typically you would implement the JobSystem interface yourself and let Jolt Physics run on top of your own job scheduler. JobSystemThreadPool is an example implementation.

	const uint cMaxBodies = 1024;							// This is the max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
	const uint cNumBodyMutexes = 0;							// This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
	const uint cMaxBodyPairs = 1024;						// This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
	const uint cMaxContactConstraints = 1024;				// This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this number then these contacts will be ignored and bodies will start interpenetrating / fall through the world. This value is low because this is a simple test. For a real project use something in the order of 10240.

	// INITIALIZE
	BPLayerInterfaceImpl broad_phase_layer_interface;									// Create mapping table from object layer to broadphase layer. As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;				// Create class that filters object vs broadphase layers. As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	ObjectLayerPairFilterImpl object_vs_object_layer_filter;							// Create class that filters object vs object layers. As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	PhysicsSystem physics_system;
	physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_vs_object_layer_filter); // And initialize!
	MyBodyActivationListener body_activation_listener;									// A body activation listener gets notified when bodies activate and go to sleep. Note that this is called from a job so whatever you do here needs to be thread safe. Registering one is entirely optional.
	physics_system.SetBodyActivationListener(&body_activation_listener);
	MyContactListener contact_listener;													// A contact listener gets notified when bodies (are about to) collide, and when they separate again. Note that this is called from a job so whatever you do here needs to be thread safe. Registering one is entirely optional.
	physics_system.SetContactListener(&contact_listener);
	BodyInterface &body_interface = physics_system.GetBodyInterface();					// The main way to interact with the bodies in the physics system is through the body interface. There is a locking and a non-locking variant of this. We're going to use the locking version (even though we're not planning to access bodies from multiple threads)

	// CREATE THE FLOOR
	BoxShapeSettings floor_shape_settings(Vec3(100.0f, 2.0f, 100.0f));	// Create the settings for the collision volume (the shape). This will be the floor
	floor_shape_settings.SetEmbedded();														// A ref counted object on the stack (base class RefTarget) should be marked as such to prevent it from being freed when its reference count goes to 0.
	ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();			// Create the shape
	ShapeRefC floor_shape = floor_shape_result.Get();										// We don't expect an error here, but you can check floor_shape_result for HasError() / GetError()
	BodyCreationSettings floor_settings(floor_shape, RVec3(0.0_r, -10.0_r, 0.0_r), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING); // Create the settings for the body itself. Note that here you can also set other properties like the restitution / friction.
	Body *floor = body_interface.CreateBody(floor_settings); 								// Create the actual rigid body. Note that if we run out of bodies this can return nullptr
	body_interface.AddBody(floor->GetID(), EActivation::DontActivate);						// Add it to the world

	BodyCreationSettings sphere_settings(new SphereShape(0.5f), RVec3(0.0_r, 2.0_r, 0.0_r), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING); // Now create a dynamic body to bounce on the floor. Note that this uses the shorthand version of creating and adding a body to the world
	BodyID sphere_id = body_interface.CreateAndAddBody(sphere_settings, EActivation::Activate);

	body_interface.SetLinearVelocity(sphere_id, Vec3(0.0f, 500.0f, 0.0f));	// Now you can interact with the dynamic body, in this case we're going to give it a velocity. (note that if we had used CreateBody then we could have set the velocity straight on the body before adding it to the physics system)

	physics_system.OptimizeBroadPhase();															// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies) You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation. Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.

	uint step = 0;
	const int targetFPS = 120;
	while (glfwWindowShouldClose(window) == 0)
	{
		++step;

		RVec3 position = body_interface.GetCenterOfMassPosition(sphere_id);
		Vec3 velocity = body_interface.GetLinearVelocity(sphere_id);
		std::cout << "Step " << step << ": Position = (" << position.GetX() << ", " << position.GetY() << ", " << position.GetZ() << "), Velocity = (" << velocity.GetX() << ", " << velocity.GetY() << ", " << velocity.GetZ() << ")" << std::endl;

		// Step the world
		physics_system.Update(1.0f/60.0f, 1, &temp_allocator, &job_system); // We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		Display();
		glFlush();
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	body_interface.RemoveBody(sphere_id);
	body_interface.DestroyBody(sphere_id);
	body_interface.RemoveBody(floor->GetID());
	body_interface.DestroyBody(floor->GetID());

	UnregisterTypes();											// Unregisters all types with the factory and cleans up the default material

	delete Factory::sInstance;
	Factory::sInstance = nullptr;

	glfwTerminate();
	return 0;
}