//! 2D physics ECS

pub use ecs::collide::prelude2d::*;
pub use ecs::physics::{DeltaTime, WithLazyRigidBody, WithRigidBody};
pub use physics::prelude2d::*;

use cgmath::{Basis2, Point2};
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

/// Utility method for registering 2D physics and collision components and resources with
/// [`specs::World`](https://docs.rs/specs/0.9.5/specs/struct.World.html).
///
/// # Parameters
///
/// - `world`: The [world](https://docs.rs/specs/0.9.5/specs/struct.World.html)
/// to register components/resources in.
///
/// # Type parameters
///
/// - `Y`: Collision shape type, see `Collider`
pub fn register_physics<Y>(world: &mut World)
where
    Y: Default + Send + Sync + 'static,
{
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass2>();
    world.register::<Velocity2>();
    world.register::<NextFrame<Velocity2>>();
    world.register::<RigidBody>();
    world.register::<ForceAccumulator2>();
    register_collision::<BodyPose2, Y>(world);
}
