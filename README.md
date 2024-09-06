#README

## Project organization
* Nothing here yet

## TODO
* https://github.com/nlohmann/json
* render objects and make sure rendering is consistent with physics simulation
* implement GLTF model loading
* figure out how sounds will work

### Pseudocode for threads and interpolation
This shouldn't be hard and isn't a priority for now.
```
main 
    simulation_thread()
    rendering_thread()

rendering_thread
    loop
        alpha = (current_time - last_physics_step_time) / physics_step_duration
        interpolate_state(alpha)
        render
        wait 1/120 seconds

simulation_thread
    loop
        step physics
        last_physics_step_time = current_time
        wait 1/60 seconds

interpolate_state(alpha)
    for each object
        interpolated_position = previous_position + (current_position - previous_position) * alpha
        set object rendering position to interpolated_position
```