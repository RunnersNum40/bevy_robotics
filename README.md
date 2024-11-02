# Bevy Robotics Simulation

This is a project to experiment with robotics simulation using the [Bevy game engine](https://bevyengine.org/) and the [Avian physics engine](https://github.com/Jondolf/avian) in Rust.

Currently, I'm just working on setting up a simple physically accurate simulation from a URDF file.
I've copied a URDF and meshes from [the Low-Cost Robot Arm project](https://github.com/AlexanderKoch-Koch/low_cost_robot) (MIT License).

My main motivation is to provide Rust native tools to replace the ROS ecosystem for robotics simulation.
I believe that Rust is a much better language for robotics development than C++ and that the Rust ecosystem can provide a better alternative to the ROS ecosystem.

## Running the project

1. Clone the repo
2. Change to the project directory
3. Run `cargo run`
