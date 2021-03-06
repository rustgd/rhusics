//! Type wrappers and convenience functions for 3D collision detection

pub use collision::algorithm::minkowski::GJK3;
pub use collision::primitive::{ConvexPolyhedron, Cuboid, Particle3, Sphere};

use cgmath::{Point3, Quaternion};
use collision::algorithm::broad_phase::BruteForce;
use collision::primitive::Primitive3;
use collision::Aabb3;

use collide::*;
use BodyPose;

/// Collision shape for 3D, see [`CollisionShape`](../collide/struct.CollisionShape.html) for more
/// information
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
/// - `Y`: Collider type, see `Collider` for more information
pub type CollisionShape3<S, T, Y = ()> = CollisionShape<Primitive3<S>, T, Aabb3<S>, Y>;

/// Broad phase brute force algorithm for 3D, see
/// [`BruteForce`](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce3 = BruteForce;

/// Broad phase sweep and prune algorithm
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type SweepAndPrune3<S> = ::collision::algorithm::broad_phase::SweepAndPrune3<S, Aabb3<S>>;

/// Body pose transform for 3D, see [`BodyPose`](../struct.BodyPose.html) for more information.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type BodyPose3<S> = BodyPose<Point3<S>, Quaternion<S>>;
