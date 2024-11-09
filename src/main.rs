use avian_motors::motor::{MotorTotalRotation, TargetRotation};
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_fps_counter::FpsCounterPlugin;
use bevy_urdf::{Robot, RobotSpawnerPlugin, URDFJoint};

mod camera;
mod world;

use camera::CameraPlugin;
use world::WorldPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            FpsCounterPlugin,
            WorldPlugin,
            CameraPlugin,
            RobotSpawnerPlugin,
            EguiPlugin,
        ))
        .add_systems(Startup, spawn_test_robot)
        .add_systems(Update, display_joint_info_ui)
        .run();
}

fn spawn_test_robot(mut commands: Commands) {
    let robot = Robot::new(
        "simple.urdf",
        Transform {
            translation: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            scale: Vec3::ONE,
        },
    );

    commands.spawn(robot);
}

fn display_joint_info_ui(
    mut egui_context: EguiContexts,
    query: Query<(&TargetRotation, &MotorTotalRotation, &URDFJoint)>,
) {
    egui::Window::new("Joint Information").show(egui_context.ctx_mut(), |ui| {
        for (target, current, joint) in query.iter() {
            ui.separator();
            ui.label(format!("{:?}", joint.name));
            ui.label(format!("Target Rotation: {:.2}", target.0));
            ui.label(format!("Current Rotation: {:.2}", current.0));
        }
    });
}
