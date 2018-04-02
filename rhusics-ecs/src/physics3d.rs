//! 3D physics ECS

pub use collide3d::*;
pub use core::physics3d::*;

use cgmath::{BaseFloat, Matrix3, Point3, Quaternion, Vector3};
use collision::Aabb3;
use collision::dbvt::TreeValueWrapped;
use collision::primitive::Primitive3;
use core::{Collider, Pose};
use specs::{Component, Entity, World};

use physics::{ContactResolutionSystem, CurrentFrameUpdateSystem, NextFrameSetupSystem,
              RigidBodyParts};
use resources::WithRhusics;

/// Current frame integrator system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform type (`BodyPose3` or similar)
pub type CurrentFrameUpdateSystem3<S, T> =
    CurrentFrameUpdateSystem<Point3<S>, Quaternion<S>, Vector3<S>, T>;

/// Resolution system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform type (`BodyPose3` or similar)
pub type ContactResolutionSystem3<S, T> =
    ContactResolutionSystem<Point3<S>, Quaternion<S>, Matrix3<S>, Vector3<S>, Vector3<S>, T>;

/// Next frame setup system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform type (`BodyPose3` or similar)
pub type NextFrameSetupSystem3<S, T> =
    NextFrameSetupSystem<Point3<S>, Quaternion<S>, Matrix3<S>, Vector3<S>, T>;

/// SystemData for 3D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform type (`BodyPose3` or similar)
/// - `Y`: Collision shape type, see `Collider`
pub type RigidBodyParts3<'a, S, T, Y> = RigidBodyParts<
    'a,
    Primitive3<S>,
    Y,
    Quaternion<S>,
    Vector3<S>,
    Vector3<S>,
    Matrix3<S>,
    Aabb3<S>,
    T,
>;

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
/// - `T`: Transform type (`BodyPose3` or similar)
pub fn register_physics<S, Y, T>(world: &mut World)
where
    S: BaseFloat + Send + Sync + 'static,
    Y: Collider + Default + Send + Sync + 'static,
    T: Pose<Point3<S>, Quaternion<S>> + Clone + Component + Send + Sync + 'static,
{
    world.register_physics::<
        Primitive3<S>,
        Aabb3<S>,
        Quaternion<S>,
        TreeValueWrapped<Entity, Aabb3<S>>,
        Y,
        Vector3<S>,
        Vector3<S>,
        Matrix3<S>,
        T
    >();
}
