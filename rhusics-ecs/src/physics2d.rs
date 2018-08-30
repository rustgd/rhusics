//! 2D physics ECS

pub use collide2d::*;
pub use core::physics2d::*;
pub use physics::setup_dispatch_2d;

use cgmath::{Basis2, Point2, Vector2};
use collision::primitive::Primitive2;
use collision::Aabb2;

use physics::{
    ContactResolutionSystem, CurrentFrameUpdateSystem, DeltaTime, NextFrameSetupSystem,
    PhysicalEntityParts,
};

/// Current frame integrator system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
pub type CurrentFrameUpdateSystem2<S, T> = CurrentFrameUpdateSystem<Point2<S>, Basis2<S>, S, T>;

/// Resolution system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform type (`BodyPose2` or similar)
pub type ContactResolutionSystem2<S, T> = ContactResolutionSystem<Point2<S>, Basis2<S>, S, S, S, T>;

/// Next frame setup system for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform type (`BodyPose2` or similar)
pub type NextFrameSetupSystem2<S, T> =
    NextFrameSetupSystem<Point2<S>, Basis2<S>, S, S, T, DeltaTime<S>>;

/// SystemData for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform type (`BodyPose2` or similar)
/// - `Y`: Collision shape type, see `Collider`
pub type PhysicalEntityParts2<'a, S, T, Y> =
    PhysicalEntityParts<'a, Primitive2<S>, Y, Basis2<S>, Vector2<S>, S, S, Aabb2<S>, T>;
