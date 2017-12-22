//! 2D physics ECS

pub use ecs::collide::prelude2d::*;
pub use ecs::physics::{DeltaTime, WithRigidBody, WithLazyRigidBody};
pub use physics::prelude2d::*;

use cgmath::{Basis2, Point2};
use shrev::EventChannel;
use specs::World;

use {NextFrame, Real};
use ecs::physics::{ContactResolutionSystem, ImpulseSolverSystem, NextFrameSetupSystem};

/// Current frame integrator system for 2D
pub type ImpulseSolverSystem2 = ImpulseSolverSystem<Point2<Real>, Basis2<Real>, Real>;

/// Resolution system for 2D
pub type ContactResolutionSystem2 = ContactResolutionSystem<
    Point2<Real>,
    Basis2<Real>,
    Real,
    Real,
    Real,
>;

/// Next frame setup system for 2D
pub type NextFrameSetupSystem2 = NextFrameSetupSystem<Point2<Real>, Basis2<Real>, Real, Real>;

/// Register required components and resources in world
pub fn world_physics_register<Y>(world: &mut World)
where
    Y: Default + Send + Sync + 'static,
{
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass2>();
    world.register::<Velocity2>();
    world.register::<NextFrame<Velocity2>>();
    world.register::<RigidBody>();
    world.register::<ForceAccumulator2>();
    world.add_resource(EventChannel::<ContactEvent2>::new());
    world_register::<BodyPose2, Y>(world);
}

/// Register required components and resources in world
pub fn world_physics_register_with_spatial<Y>(world: &mut World)
where
    Y: Default + Send + Sync + 'static,
{
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass2>();
    world.register::<Velocity2>();
    world.register::<RigidBody>();
    world.register::<ForceAccumulator2>();
    world.register::<NextFrame<Velocity2>>();
    world.add_resource(EventChannel::<ContactEvent2>::new());
    world_register_with_spatial::<BodyPose2, Y>(world);
}
