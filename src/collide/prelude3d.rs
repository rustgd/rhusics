//! Type wrappers and convenience functions for 3D collision detection

pub use collide::{Collider, CollisionMode, CollisionStrategy};
pub use collision::algorithm::minkowski::GJK3;
pub use collision::primitive::{ConvexPolyhedron, Cuboid, Particle3, Sphere};

use cgmath::{Point3, Quaternion};
use collision::algorithm::broad_phase::BruteForce;
use collision::primitive::Primitive3;

use {BodyPose, Real};
use collide::*;

/// Collision shape for 3D, see [CollisionShape](../collide/struct.CollisionShape.html) for more
/// information
pub type CollisionShape3<T, Y = ()> = CollisionShape<Primitive3<Real>, T, Y>;

/// Broad phase brute force algorithm for 3D, see
/// [BruteForce](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce3 = BruteForce;

/// Broad phase sweep and prune algorithm
pub type SweepAndPrune3 = ::collision::algorithm::broad_phase::SweepAndPrune3<Real>;

/// Body pose transform for 3D, see [BodyPose](../struct.BodyPose.html) for more information.
pub type BodyPose3 = BodyPose<Point3<Real>, Quaternion<Real>>;
