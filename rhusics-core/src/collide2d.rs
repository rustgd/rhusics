//! Type wrappers and convenience functions for 2D collision detection

pub use collision::algorithm::minkowski::GJK2;
pub use collision::primitive::{Circle, ConvexPolygon, Particle2, Rectangle};

use cgmath::{Basis2, Point2};
use collision::Aabb2;
use collision::algorithm::broad_phase::BruteForce;
use collision::primitive::Primitive2;

use BodyPose;
use collide::*;

/// Collision shape for 2D, see [CollisionShape](../collide/struct.CollisionShape.html) for more
/// information
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
/// - `Y`: Collider type, see `Collider` for more information
pub type CollisionShape2<S, T, Y = ()> = CollisionShape<Primitive2<S>, T, Aabb2<S>, Y>;

/// Broad phase brute force algorithm for 2D, see
/// [BruteForce](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce2 = BruteForce;

/// Broad phase sweep and prune algorithm
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type SweepAndPrune2<S> = ::collision::algorithm::broad_phase::SweepAndPrune2<S, Aabb2<S>>;

/// Body pose transform for 2D, see [BodyPose](../struct.BodyPose.html) for more information.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type BodyPose2<S> = BodyPose<Point2<S>, Basis2<S>>;
