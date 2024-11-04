use bevy::prelude::*;
use bevy_fps_counter::FpsCounterPlugin;

mod world;
use world::WorldPlugin;

mod camera;
use camera::CameraPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            FpsCounterPlugin,
            WorldPlugin,
            CameraPlugin,
        ))
        .run();
}

