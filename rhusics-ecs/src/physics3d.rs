//! 3D physics ECS

pub use collide3d::*;
pub use core::physics3d::*;

use cgmath::{BaseFloat, Matrix3, Point3, Quaternion, Vector3};
use collision::Aabb3;
use collision::dbvt::TreeValueWrapped;
use collision::primitive::Primitive3;
use core::Collider;
use specs::{Entity, World};

use physics::{ContactResolutionSystem, CurrentFrameUpdateSystem, NextFrameSetupSystem};
use resources::WithRhusics;

/// Current frame integrator system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type CurrentFrameUpdateSystem3<S> =
    CurrentFrameUpdateSystem<Point3<S>, Quaternion<S>, Vector3<S>>;

/// Resolution system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type ContactResolutionSystem3<S> =
    ContactResolutionSystem<Point3<S>, Quaternion<S>, Matrix3<S>, Vector3<S>, Vector3<S>>;

/// Next frame setup system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type NextFrameSetupSystem3<S> =
    NextFrameSetupSystem<Point3<S>, Quaternion<S>, Matrix3<S>, Vector3<S>>;

/// Utility method for registering 3D physics and collision components and resources with
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
pub fn register_physics<S, Y>(world: &mut World)
where
    S: BaseFloat + Send + Sync + 'static,
    Y: Collider + Default + Send + Sync + 'static,
{
    world.register_physics::<
        Primitive3<S>,
        Aabb3<S>,
        Quaternion<S>,
        TreeValueWrapped<Entity, Aabb3<S>>,
        Y,
        Vector3<S>,
        Vector3<S>,
        Matrix3<S>
    >();
}
