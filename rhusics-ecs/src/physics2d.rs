//! 2D physics ECS

pub use collide2d::*;
pub use core::physics2d::*;

use cgmath::{BaseFloat, Basis2, Point2, Vector2};
use collision::Aabb2;
use collision::dbvt::TreeValueWrapped;
use collision::primitive::Primitive2;
use core::{Collider, Pose};
use specs::{Component, Entity, World};

use physics::{ContactResolutionSystem, CurrentFrameUpdateSystem, NextFrameSetupSystem};
use resources::WithRhusics;

/// Current frame integrator system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type CurrentFrameUpdateSystem2<S, T> = CurrentFrameUpdateSystem<Point2<S>, Basis2<S>, S, T>;

/// Resolution system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type ContactResolutionSystem2<S, T> = ContactResolutionSystem<Point2<S>, Basis2<S>, S, S, S, T>;

/// Next frame setup system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type NextFrameSetupSystem2<S, T> = NextFrameSetupSystem<Point2<S>, Basis2<S>, S, S, T>;

/// Utility method for registering 2D physics and collision components and resources with
/// [`specs::World`](https://docs.rs/specs/0.9.5/specs/struct.World.html).
///
/// # Parameters
///
/// - `world`: The [world](https://docs.rs/specs/0.9.5/specs/struct.World.html)
/// to register components/resources in.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `Y`: Collision shape type, see `Collider`
pub fn register_physics<S, Y, T>(world: &mut World)
where
    S: BaseFloat + Send + Sync + 'static,
    Y: Collider + Default + Send + Sync + 'static,
    T: Pose<Point2<S>, Basis2<S>> + Clone + Component + Send + Sync + 'static,
{
    world.register_physics::<
        Primitive2<S>,
        Aabb2<S>,
        Basis2<S>,
        TreeValueWrapped<Entity, Aabb2<S>>,
        Y,
        Vector2<S>,
        S,
        S,
        T
    >();
}
