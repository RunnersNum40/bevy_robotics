use avian3d::prelude::*;
use avian_motors::motor::{
    MotorDerivativeGain, MotorIntegralGain, MotorMaxAngularVelocity, MotorMaxTorque, MotorPlugin,
    MotorProportionalGain, RevoluteMotorBundle, TargetRotation,
};
use bevy::prelude::*;
use std::collections::HashMap;
use std::f64::consts::PI;
use urdf_rs::{Geometry, Pose};
use uuid::Uuid;

#[derive(Component, Clone, Debug)]
pub struct URDFLink;

#[derive(Component, Clone, Debug)]
pub struct URDFCollider;

#[derive(Component, Clone, Debug)]
pub struct URDFVisual;

#[derive(Component, Clone, Debug)]
pub struct URDFJoint {
    pub name: String,
}

#[derive(Component, Clone, Debug)]
pub struct Robot {
    pub urdf_path: String,
    pub base_transform: Transform,
}

impl Robot {
    pub fn new(urdf_path: &str, base_transform: Transform) -> Self {
        Self {
            urdf_path: urdf_path.to_string(),
            base_transform,
        }
    }
}

pub struct RobotSpawnerPlugin;

impl Plugin for RobotSpawnerPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            bevy_stl::StlPlugin,
            PhysicsPlugins::default(),
            // PhysicsDebugPlugin::default(),
            MotorPlugin {
                substep_count: None,
                remove_dampening: false,
                ..Default::default()
            },
        ))
        .add_systems(FixedUpdate, spawn_robot_system)
        .add_systems(PostProcessCollisions, ignore_collision)
        .insert_gizmo_config(
            PhysicsGizmos {
                axis_lengths: None,
                contact_point_color: Some(Color::srgb(1.0, 0.0, 0.0)),
                ..default()
            },
            GizmoConfig::default(),
        )
        .insert_resource(SubstepCount(300));
    }
}

fn spawn_robot_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    query: Query<(Entity, &Robot), Added<Robot>>,
) {
    for (entity, component) in query.iter() {
        let robot_params = RobotParams {
            urdf_path: &component.urdf_path,
            base_transform: component.base_transform,
        };
        spawn_robot(
            robot_params,
            &mut commands,
            &mut meshes,
            &asset_server,
            &mut materials,
        );

        commands.entity(entity).despawn();
    }
}

struct RobotParams<'a> {
    urdf_path: &'a str,
    base_transform: Transform,
}

fn spawn_robot(
    params: RobotParams,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let robot = load_robot_urdf(params.urdf_path);
    let mut link_entities = HashMap::new();
    let mut link_transforms = HashMap::new();

    for link in robot.links {
        let link_entity_id = create_robot_link(
            &link,
            params.base_transform,
            commands,
            &mut link_entities,
            &mut link_transforms,
        );
        add_visuals_to_link(
            &link,
            link_entity_id,
            commands,
            meshes,
            asset_server,
            materials,
        );
        add_collisions_to_link(&link, link_entity_id, commands, meshes, asset_server);
    }

    setup_robot_joints(
        &robot.joints,
        &link_entities,
        &mut link_transforms,
        commands,
    );
}

fn load_robot_urdf(urdf_path: &str) -> urdf_rs::Robot {
    urdf_rs::read_file(urdf_path).expect("Failed to read URDF file")
}

fn create_robot_link(
    link: &urdf_rs::Link,
    base_transform: Transform,
    commands: &mut Commands,
    link_entities: &mut HashMap<String, Entity>,
    link_transforms: &mut HashMap<String, Transform>,
) -> Entity {
    let initial_transform = if link.name == "base_link" {
        base_transform
    } else {
        Transform::IDENTITY
    };
    let mass_properties = link_to_mass_properties(link);

    let link_entity = commands.spawn((
        initial_transform,
        match link.name.as_str() {
            "base_link" => RigidBody::Kinematic,
            _ => RigidBody::Dynamic,
        },
        mass_properties,
        InheritedVisibility::VISIBLE,
        GlobalTransform::IDENTITY,
        URDFLink,
    ));
    let link_entity_id = link_entity.id();
    link_entities.insert(link.name.clone(), link_entity_id);
    link_transforms.insert(link.name.clone(), initial_transform);

    link_entity_id
}

fn add_visuals_to_link(
    link: &urdf_rs::Link,
    entity_id: Entity,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    for visual in &link.visual {
        let (mesh_handle, material_handle) =
            visual_to_mesh_and_material(visual, meshes, asset_server, materials);
        commands.entity(entity_id).with_children(|parent| {
            parent.spawn((
                PbrBundle {
                    mesh: mesh_handle,
                    material: material_handle,
                    transform: urdf_to_transform(&visual.origin, &Some(visual.geometry.clone())),
                    ..Default::default()
                },
                URDFVisual,
            ));
        });
    }
}

fn add_collisions_to_link(
    link: &urdf_rs::Link,
    entity_id: Entity,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
) {
    for collision in &link.collision {
        let mesh_handle = collision_to_mesh(collision, meshes, asset_server);
        let link_id = Uuid::new_v4();
        let shift_amount = (link_id.as_u128() % 32) as u32;
        let layer_mask = 1 << shift_amount;
        commands.entity(entity_id).with_children(|parent| {
            parent.spawn((
                mesh_handle,
                urdf_to_transform(&collision.origin, &Some(collision.geometry.clone())),
                ColliderConstructor::ConvexDecompositionFromMeshWithConfig(VhacdParameters {
                    resolution: 128,
                    ..Default::default()
                }),
                URDFCollider,
                CollisionLayers::new(layer_mask, !layer_mask),
            ));
        });
    }
}

fn setup_robot_joints(
    joints: &[urdf_rs::Joint],
    link_entities: &HashMap<String, Entity>,
    link_transforms: &mut HashMap<String, Transform>,
    commands: &mut Commands,
) {
    for (idx, joint) in joints.into_iter().enumerate() {
        if let (Some(parent_entity), Some(child_entity)) = (
            link_entities.get(&joint.parent.link),
            link_entities.get(&joint.child.link),
        ) {
            let joint_transform = urdf_to_transform(&joint.origin, &None);
            if let Some(parent_transform) = link_transforms.get(&joint.parent.link) {
                let accumulated_transform = *parent_transform * joint_transform;
                link_transforms.insert(joint.child.link.clone(), accumulated_transform);
                commands.entity(*child_entity).insert(accumulated_transform);
            }
            urdf_to_joint(commands, *parent_entity, *child_entity, joint, idx <= 1);
        }
    }
}

fn link_to_mass_properties(link: &urdf_rs::Link) -> MassPropertiesBundle {
    MassPropertiesBundle {
        mass: Mass(link.inertial.mass.value),
        inertia: Inertia(avian3d::math::Matrix3 {
            x_axis: Vec3::new(
                link.inertial.inertia.ixx as f32,
                link.inertial.inertia.ixy as f32,
                link.inertial.inertia.ixz as f32,
            )
            .into(),
            y_axis: Vec3::new(
                link.inertial.inertia.ixy as f32,
                link.inertial.inertia.iyy as f32,
                link.inertial.inertia.iyz as f32,
            )
            .into(),
            z_axis: Vec3::new(
                link.inertial.inertia.ixz as f32,
                link.inertial.inertia.iyz as f32,
                link.inertial.inertia.izz as f32,
            )
            .into(),
        }),
        center_of_mass: CenterOfMass(
            Vec3::new(
                link.inertial.origin.xyz[0] as f32,
                link.inertial.origin.xyz[1] as f32,
                link.inertial.origin.xyz[2] as f32,
            )
            .into(),
        ),
        ..Default::default()
    }
}

fn visual_to_mesh_and_material(
    visual: &urdf_rs::Visual,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> (Handle<Mesh>, Handle<StandardMaterial>) {
    let material_handle = create_material(&visual.material, materials);
    match &visual.geometry {
        Geometry::Mesh { filename, .. } => (asset_server.load(filename), material_handle),
        Geometry::Box { size } => (
            meshes.add(Mesh::from(Cuboid::new(
                size[0] as f32,
                size[1] as f32,
                size[2] as f32,
            ))),
            material_handle,
        ),
        Geometry::Cylinder { radius, length } => (
            meshes.add(Mesh::from(Cylinder {
                radius: *radius as f32,
                half_height: (*length as f32) / 2.0,
            })),
            material_handle,
        ),
        Geometry::Sphere { radius } => (
            meshes.add(Mesh::from(Sphere {
                radius: *radius as f32,
            })),
            material_handle,
        ),
        _ => {
            error!("Unsupported geometry type: {:?}", visual.geometry);
            (Handle::default(), Handle::default())
        }
    }
}

fn collision_to_mesh(
    collision: &urdf_rs::Collision,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
) -> Handle<Mesh> {
    match &collision.geometry {
        Geometry::Mesh { filename, .. } => asset_server.load(filename),
        Geometry::Box { size } => meshes.add(Mesh::from(Cuboid::new(
            size[0] as f32,
            size[1] as f32,
            size[2] as f32,
        ))),
        Geometry::Cylinder { radius, length } => meshes.add(Mesh::from(Cylinder {
            radius: *radius as f32,
            half_height: (*length as f32) / 2.0,
        })),
        Geometry::Sphere { radius } => meshes.add(Mesh::from(Sphere {
            radius: *radius as f32,
        })),
        _ => {
            error!("Unsupported geometry type: {:?}", collision.geometry);
            Handle::default()
        }
    }
}

fn create_material(
    urdf_material: &Option<urdf_rs::Material>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> Handle<StandardMaterial> {
    let color = urdf_material
        .as_ref()
        .and_then(|material| material.color.as_ref())
        .map_or_else(
            || Color::srgba(0.7, 0.7, 0.7, 1.0),
            |urdf_color| {
                Color::srgba(
                    urdf_color.rgba[0] as f32,
                    urdf_color.rgba[1] as f32,
                    urdf_color.rgba[2] as f32,
                    urdf_color.rgba[3] as f32,
                )
            },
        );

    materials.add(StandardMaterial {
        base_color: color,
        metallic: 0.7,
        ..Default::default()
    })
}

fn urdf_to_transform(origin: &Pose, geometry: &Option<Geometry>) -> Transform {
    let scale = geometry
        .as_ref()
        .and_then(|g| {
            if let Geometry::Mesh {
                scale: Some(mesh_scale),
                ..
            } = g
            {
                Some(Vec3::new(
                    mesh_scale[0] as f32,
                    mesh_scale[1] as f32,
                    mesh_scale[2] as f32,
                ))
            } else {
                None
            }
        })
        .unwrap_or(Vec3::ONE);

    Transform {
        translation: Vec3::new(
            origin.xyz[0] as f32,
            origin.xyz[1] as f32,
            origin.xyz[2] as f32,
        ),
        rotation: Quat::from_euler(
            EulerRot::XYZ,
            origin.rpy[0] as f32,
            origin.rpy[1] as f32,
            origin.rpy[2] as f32,
        ),
        scale,
    }
}

fn urdf_to_joint(
    commands: &mut Commands,
    entity1: Entity,
    entity2: Entity,
    urdf_joint: &urdf_rs::Joint,
    add_motor: bool,
) {
    let axis = Vec3::from(urdf_joint.axis.xyz.map(|v| v as f32));
    let dynamics = urdf_joint.dynamics.clone().unwrap_or(urdf_rs::Dynamics {
        damping: 10.0,
        friction: 0.0,
    });

    let anchor = Vec3::new(
        urdf_joint.origin.xyz[0] as f32,
        urdf_joint.origin.xyz[1] as f32,
        urdf_joint.origin.xyz[2] as f32,
    );

    match urdf_joint.joint_type {
        urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous => {
            let joint = RevoluteJoint::new(entity1, entity2)
                .with_aligned_axis(axis.into())
                .with_local_anchor_1(anchor.into())
                .with_angular_velocity_damping(dynamics.damping)
                .with_compliance(0.0);

            let joint = if let urdf_rs::JointType::Revolute = urdf_joint.joint_type {
                joint.with_angle_limits(urdf_joint.limit.lower, urdf_joint.limit.upper)
            } else {
                joint.with_angle_limits(-PI, PI)
            };

            let motor = (
                TargetRotation(Vec3::ZERO.into()),
                RevoluteMotorBundle {
                    stiffness: MotorProportionalGain(5.0),
                    damping: MotorDerivativeGain(0.1),
                    integral_gain: MotorIntegralGain(0.1),
                    max_angular_velocity: MotorMaxAngularVelocity(None),
                    max_torque: MotorMaxTorque(Some(0.25)),
                    ..Default::default()
                },
            );

            if add_motor {
                commands.spawn((
                    joint,
                    motor,
                    URDFJoint {
                        name: urdf_joint.name.clone(),
                    },
                ));
            } else {
                commands.spawn((
                    joint,
                    URDFJoint {
                        name: urdf_joint.name.clone(),
                    },
                ));
            }
        }
        urdf_rs::JointType::Fixed => {
            commands.spawn(FixedJoint::new(entity1, entity2));
        }
        _ => error!("Unsupported joint type: {:?}", urdf_joint.joint_type),
    }
}

fn ignore_collision(mut collisions: ResMut<Collisions>, query: Query<&URDFCollider>) {
    collisions.retain(|contacts| {
        let entity1_is_urdf = query.get(contacts.entity1).is_ok();
        let entity2_is_urdf = query.get(contacts.entity2).is_ok();

        if entity1_is_urdf && entity2_is_urdf {
            let max_penetration_amount = max_penetration(contacts);

            max_penetration_amount > 0.004;
            false
        } else {
            true
        }
    });
}

fn max_penetration(contacts: &mut Contacts) -> f64 {
    contacts
        .manifolds
        .iter_mut()
        .fold(0.0, |max_penetration, manifold| {
            manifold
                .contacts
                .iter_mut()
                .fold(max_penetration, |max_penetration, contact| {
                    f64::max(max_penetration, contact.penetration)
                })
        })
}
