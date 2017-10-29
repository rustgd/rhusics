//! Type wrappers and convenience functions for 2D collision detection

pub use collide::{CollisionMode, CollisionStrategy};
pub use collision::algorithm::minkowski::GJK2;
pub use collision::primitive::{Circle, ConvexPolygon, Particle2, Rectangle};

use cgmath::{Basis2, Point2};
use collision::algorithm::broad_phase::BruteForce;
use collision::primitive::Primitive2;

use {BodyPose, Real};
use collide::*;

/// Collision shape for 2D, see [CollisionShape](../collide/struct.CollisionShape.html) for more
/// information
pub type CollisionShape2<T> = CollisionShape<Primitive2<Real>, T>;

/// Broad phase brute force algorithm for 2D, see
/// [BruteForce](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce2 = BruteForce;

/// Broad phase sweep and prune algorithm
pub type SweepAndPrune2 = ::collision::algorithm::broad_phase::SweepAndPrune2<Real>;

/// Body pose transform for 2D, see [BodyPose](../struct.BodyPose.html) for more information.
pub type BodyPose2 = BodyPose<Point2<Real>, Basis2<Real>>;
