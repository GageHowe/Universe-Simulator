# DESIGN

    NON_MOVING - Layer for all static objects.
    MOVING - Layer for all regular dynamic bodies.
    DEBRIS - Layer for all debris dynamic bodies, we want to test these only against the static geometry because we want to save some simulation cost.
    BULLET - Layer for high detail collision bodies that we attach to regular dynamic bodies. These are not used for simulation but we want extra precision when we shoot with bullets.
    WEAPON - This is a query layer so we don't create any bodies with this layer but we use it when doing ray cast querying for our weapon system.

Spaceship simulation in 3D, with a high amount of realism. This includes:
* Bullets fired affect the velocity of the ship
* If something moves, it will continue to move
* Sounds don't travel in space.

### Libraries used
* Model loading: [Assimp](https://learnopengl.com/Model-Loading/Assimp)
* Physics: [Jolt](https://jrouwe.github.io/JoltPhysics/)
* Sound: fMod https://www.fmod.com/download#fmodengine or SDL3 audio https://wiki.libsdl.org/SDL3/CategoryAudio or miniaudio https://github.com/mackron/miniaudio
### Software Design Philosophy
Use modules and namespaces. OOP-based classes like GameState, PhysicsEngine, Graphics etc are an over-engineered solution, and we won't need to instantiate multiple of these at any time.

I'm not making a game engine. There's no need to create abstractions over library functions. And I'm using the simplest libraries for the job.

### Physics
Scale is everything. Massive ships, planets, etc. luckily, Jolt's optimized 64-bit simulation offers vastly larger precision at a 5% premium over 32-bit simulation.

High precision 64-bit floats will be used while the player is flying in "normal mode." If the player must go outside the bounds of the physics world, the "world" resets and the origin is set at the location of the player. Remember all physics is expressed in meters.

### Sound Design
Cool, sterile spaceship
https://www.google.com/search?client=firefox-b-e&q=sounds+of+a+server+farm

### Visuals
"Dark mode." Realism is a priority, but so is codebase simplicity and performance. Like the old Halo games, the emphasis is on scale and immersion, not fancy graphics
