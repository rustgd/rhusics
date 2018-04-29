//! 3D physics ECS

pub use collide3d::*;
pub use core::physics3d::*;
pub use physics::setup_dispatch_3d;

use cgmath::{Matrix3, Point3, Quaternion, Vector3};
use collision::Aabb3;
use collision::primitive::Primitive3;

use physics::{ContactResolutionSystem, CurrentFrameUpdateSystem, DeltaTime, NextFrameSetupSystem,
              RigidBodyParts};

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
    NextFrameSetupSystem<Point3<S>, Quaternion<S>, Matrix3<S>, Vector3<S>, T, DeltaTime<S>>;

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
