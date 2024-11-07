use bevy::prelude::*;
use bevy_fps_counter::FpsCounterPlugin;

mod camera;
mod robot;
mod world;

use camera::CameraPlugin;
use robot::{Robot, RobotSpawnerPlugin};
use world::WorldPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            FpsCounterPlugin,
            WorldPlugin,
            CameraPlugin,
            RobotSpawnerPlugin,
        ))
        .add_systems(Startup, spawn_test_robot)
        .run();
}

fn spawn_test_robot(mut commands: Commands) {
    let robot = Robot::new(
        "full.urdf",
        Transform {
            translation: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            scale: Vec3::ONE,
        },
    );

    commands.spawn(robot);
}
