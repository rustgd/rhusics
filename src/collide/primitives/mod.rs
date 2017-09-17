//! Collision primitives

pub use self::primitive2d::*;
pub use self::primitive3d::*;

use std::fmt::Debug;

use cgmath::prelude::*;
use collision::prelude::*;

use {Pose, Real};

pub mod primitive2d;
pub mod primitive3d;

/// Trait detailing a collision primitive. These are the building blocks for all collision shapes.
///
/// See [primitive2d](primitive2d/index.html) and [primitive3d](primitive3d/index.html)
/// for more information about supported primitives.
///
pub trait Primitive: Debug + Clone + Send + Sync {
    /// Vector type used by the primitive
    type Vector: VectorSpace<Scalar = Real> + ElementWise + Array<Element = Real>;

    /// Point type used by the primitive
    type Point: EuclideanSpace<Scalar = Real, Diff = Self::Vector> + MinMax;

    /// Bounding box type used by the primitive
    type Aabb: Aabb<Scalar = Real, Diff = Self::Vector, Point = Self::Point>
    + Clone
    + Union<Self::Aabb, Output = Self::Aabb>;

    /// Get the furthest point from the origin on the shape in a given direction.
    ///
    /// # Parameters
    ///
    /// - `direction`: The search direction in world space.
    /// - `transform`: The current local to world transform for this shape.
    ///
    /// # Returns
    ///
    /// Returns the point that is furthest away from the origin.
    ///
    /// # Type parameters
    ///
    /// - `P`: Transform type
    fn get_far_point<T>(&self, direction: &Self::Vector, transform: &T) -> Self::Point
        where
            T: Pose<Self::Point>;

    /// Get the bounding box of the primitive in local space coordinates.
    fn get_bound(&self) -> Self::Aabb;
}
