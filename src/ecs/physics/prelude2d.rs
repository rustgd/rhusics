//! 2D physics ECS

pub use ecs::collide::prelude2d::*;
pub use ecs::physics::{DeltaTime, WithLazyRigidBody, WithRigidBody};
pub use physics::prelude2d::*;

use cgmath::{BaseFloat, Basis2, Point2, Vector2};
use collision::Aabb2;
use collision::dbvt::TreeValueWrapped;
use collision::primitive::Primitive2;
use specs::{Entity, World};

use ecs::WithRhusics;
use ecs::physics::{ContactResolutionSystem, CurrentFrameUpdateSystem, NextFrameSetupSystem};

/// Current frame integrator system for 2D
pub type CurrentFrameUpdateSystem2<S> = CurrentFrameUpdateSystem<Point2<S>, Basis2<S>, S>;

/// Resolution system for 2D
pub type ContactResolutionSystem2<S> = ContactResolutionSystem<Point2<S>, Basis2<S>, S, S, S>;

/// Next frame setup system for 2D
pub type NextFrameSetupSystem2<S> = NextFrameSetupSystem<Point2<S>, Basis2<S>, S, S>;

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
pub fn register_physics<S, Y>(world: &mut World)
where
    S: BaseFloat + Send + Sync + 'static,
    Y: Collider + Default + Send + Sync + 'static,
{
    world.register_physics::<
        Primitive2<S>,
        Aabb2<S>,
        Basis2<S>,
        TreeValueWrapped<Entity, Aabb2<S>>,
        Y,
        Vector2<S>,
        S,
        S
    >();
}
