use avian3d::prelude::*;
use bevy::color;
use bevy::prelude::*;

pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (spawn_lights, spawn_floor));
    }
}

fn spawn_lights(mut commands: Commands) {
    let directional_light = DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 1000.0,
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform::from_rotation(Quat::from_rotation_x(
            -std::f32::consts::FRAC_PI_2 * 0.8,
        )),
        ..Default::default()
    };

    commands.spawn(directional_light);

    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 50.0,
    });
}

fn spawn_floor(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    const FLOOR_RADIUS: f32 = 10.0;
    const FLOOR_HEIGHT: f32 = 0.0001;

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(Cylinder::new(FLOOR_RADIUS, FLOOR_HEIGHT / 2.0))),
            material: materials.add(StandardMaterial {
                base_color: color::Color::srgb(0.0, 0.603922, 0.090196), // grass green
                ..Default::default()
            }),
            transform: Transform {
                translation: Vec3::new(0.0, -FLOOR_HEIGHT / 2.0, 0.0),
                ..Default::default()
            },
            ..Default::default()
        },
        RigidBody::Static,
        Collider::cylinder(FLOOR_RADIUS.into(), (FLOOR_HEIGHT / 2.0).into()),
    ));
}
