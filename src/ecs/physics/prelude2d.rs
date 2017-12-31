//! 2D physics ECS

pub use ecs::collide::prelude2d::*;
pub use ecs::physics::{DeltaTime, WithLazyRigidBody, WithRigidBody};
pub use physics::prelude2d::*;

use cgmath::{Basis2, Point2, Vector2};
use collision::Aabb2;
use collision::dbvt::TreeValueWrapped;
use collision::primitive::Primitive2;
use specs::{Entity, World};

use Real;
use ecs::WithRhusics;
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
    Y: Collider + Default + Send + Sync + 'static,
{
    world.register_physics::<
        Primitive2<Real>,
        Aabb2<Real>,
        Basis2<Real>,
        TreeValueWrapped<Entity, Aabb2<Real>>,
        Y,
        Vector2<Real>,
        Real,
        Real
    >();
}
