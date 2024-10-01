# DOCUMENTATION

This is for my own use, for the purpose of understanding more about OpenGL and Jolt, and improving project structure.

* https://jrouwe.github.io/JoltPhysics/
* https://learnopengl.com/
* https://www.youtube.com/playlist?list=PLPaoO-vpZnumdcb4tZc4x5Q-v7CkrQ6M-

### Notes
BodyInterface::CreateBody -> BodyInterface::AddBody -> BodyInterface::RemoveBody -> BodyInterface::DestroyBody

Simple shapes like spheres and boxes can be constructed immediately by simply new-ing them. Other shapes need to be converted into an optimized format in order to be usable in the physics simulation.

Shapes can also be saved, but that's not important now

SENSORS can be used to implement triggers that detect when an object enters their area.

---
Bodies can be connected to each other using CONSTRAINTS. The following constraints are available:
* FixedConstraint - Will attach a body to another without any degrees of freedom.
* DistanceConstraint - Will attach two bodies with a stick (removing 1 degree of freedom).
* PointConstraint - Will attach two bodies in a single point (removing 3 degrees of freedom)
* HingeConstraint - Will attach two bodies through a hinge.
* ConeConstraint - Attaches two bodies in a point and will limit the rotation within a cone.
* SliderConstraint - Attaches two bodies and allows only movement in a single translation axis (also known as prismatic constraint).
* SwingTwistConstraint - Attaches two bodies using a point constraint and a swing-twist constraint which approximates the shoulder joint of a human.
* SixDOFConstraint - The most configurable joint allows specifying per translation axis and rotation axis what the limits are.
* PathConstraint - This constraint allows attaching two bodies connected through a Hermite spline path.
* GearConstraint - This constraint connects to two hinge joints and constrains them to connect two gears.
* RackAndPinionConstraint - This constraint connects a hinge and a slider constraint to connect a rack and pinion.
* PulleyConstraint - This constraint connects two bodies through two fixed points creating something that behaves like two bodies connected through a rope.
* VehicleConstraint - This constraint adds virtual wheels or tracks to a body and allows it to behave as a vehicle.

---
### Broad Phase
When bodies are added to the PhysicsSystem, they are inserted in the broad phase (BroadPhaseQuadTree). This provides quick coarse collision detection based on the axis aligned bounding box (AABB) of a body.

# types
    NON_MOVING - Layer for all static objects.
    MOVING - Layer for all regular dynamic bodies.
    DEBRIS - Layer for all debris dynamic bodies, we want to test these only against the static geometry because we want to save some simulation cost.
    BULLET - Layer for high detail collision bodies that we attach to regular dynamic bodies. These are not used for simulation but we want extra precision when we shoot with bullets.
    WEAPON - This is a query layer so we don't create any bodies with this layer but we use it when doing ray cast querying for our weapon system.
