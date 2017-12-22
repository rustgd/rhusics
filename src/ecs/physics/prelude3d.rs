//! 3D physics ECS

pub use ecs::collide::prelude3d::*;
pub use ecs::physics::{DeltaTime, WithRigidBody, WithLazyRigidBody};
pub use physics::prelude3d::*;

use cgmath::{Matrix3, Point3, Quaternion, Vector3};
use shrev::EventChannel;
use specs::World;

use {NextFrame, Real};
use ecs::physics::{ContactResolutionSystem, ImpulseSolverSystem, NextFrameSetupSystem};

/// Current frame integrator system for 2D
pub type ImpulseSolverSystem3 = ImpulseSolverSystem<Point3<Real>, Quaternion<Real>, Vector3<Real>>;

/// Resolution system for 2D
pub type ContactResolutionSystem3 = ContactResolutionSystem<
    Point3<Real>,
    Quaternion<Real>,
    Matrix3<Real>,
    Vector3<Real>,
    Vector3<Real>,
>;

/// Next frame setup system for 2D
pub type NextFrameSetupSystem3 = NextFrameSetupSystem<
    Point3<Real>,
    Quaternion<Real>,
    Matrix3<Real>,
    Vector3<Real>,
>;

/// Register required components and resources in world
pub fn world_physics_register<Y>(world: &mut World)
where
    Y: Send + Sync + 'static,
{
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass3>();
    world.register::<Velocity3>();
    world.register::<NextFrame<Velocity3>>();
    world.register::<RigidBody>();
    world.register::<ForceAccumulator3>();
    world.add_resource(EventChannel::<ContactEvent3>::new());
    world_register::<BodyPose3, Y>(world);
}

/// Register required components and resources in world
pub fn world_physics_register_with_spatial<Y>(world: &mut World)
where
    Y: Send + Sync + 'static,
{
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass3>();
    world.register::<Velocity3>();
    world.register::<NextFrame<Velocity3>>();
    world.register::<RigidBody>();
    world.register::<ForceAccumulator3>();
    world.add_resource(EventChannel::<ContactEvent3>::new());
    world_register_with_spatial::<BodyPose3, Y>(world);
}
