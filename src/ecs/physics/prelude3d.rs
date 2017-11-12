//! 3D physics ECS

pub use ecs::collide::prelude3d::*;
pub use ecs::physics::DeltaTime;
pub use physics::prelude3d::*;

use cgmath::{Matrix3, Point3, Quaternion, Vector3};
use shrev::EventChannel;
use specs::World;

use {NextFrame, Real};
use ecs::physics::LinearSolverSystem;

/// Linear contact resolve system for 3D
pub type LinearSolverSystem3 = LinearSolverSystem<
    Point3<Real>,
    Quaternion<Real>,
    Matrix3<Real>,
    Vector3<Real>,
>;

/// Register required components and resources in world
pub fn world_physics_register(world: &mut World) {
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass3>();
    world.register::<Velocity3>();
    world.register::<NextFrame<Velocity3>>();
    world.register::<RigidBody>();
    world.register::<ForceAccumulator3>();
    world.add_resource(EventChannel::<ContactEvent3>::new());
    world_register::<BodyPose3>(world);
}

/// Register required components and resources in world
pub fn world_physics_register_with_spatial(world: &mut World) {
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass3>();
    world.register::<Velocity3>();
    world.register::<NextFrame<Velocity3>>();
    world.register::<RigidBody>();
    world.register::<ForceAccumulator3>();
    world.add_resource(EventChannel::<ContactEvent3>::new());
    world_register_with_spatial::<BodyPose3>(world);
}
