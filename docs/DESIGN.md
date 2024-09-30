# DESIGN

Make it work now, refactor it later (modularize, multithread, etc)

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
