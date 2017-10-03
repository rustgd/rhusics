//! Generic narrow phase collision detection algorithms.
//!
//! Currently only supports GJK/EPA.

use std::fmt::Debug;
use std::ops::Neg;

use cgmath::prelude::*;
use collision::{CollisionStrategy, Contact, Primitive};
use collision::algorithm::minkowski::{SimplexProcessor, EPA, GJK};
use collision::prelude::*;

use Real;
use collide::CollisionShape;

/// Base trait implemented by all narrow phase algorithms.
///
/// # Type parameters:
///
/// - `P`: collision primitive type
/// - `T`: model-to-world transform type
pub trait NarrowPhase<P, T>: Send
where
    P: Primitive,
    <P::Point as EuclideanSpace>::Diff: Debug,
{
    /// Check if two shapes collides, and give a contact manifold for the contact with the largest
    /// penetration depth.
    ///
    /// # Parameters:
    ///
    /// - `left`: the left shape
    /// - `left_transform`: model-to-world transform for the left shape
    /// - `right`: the right shape
    /// - `right_transform`: model-to-world transform for the right shape
    ///
    /// # Returns:
    ///
    /// Optionally returns the contact manifold for the contact with largest penetration depth
    fn collide(
        &self,
        left: &CollisionShape<P, T>,
        left_transform: &T,
        right: &CollisionShape<P, T>,
        right_transform: &T,
    ) -> Option<Contact<P::Point>>;
}

impl<P, T, S, E> NarrowPhase<P, T> for GJK<S, E>
where
    P: Primitive,
    <P::Point as EuclideanSpace>::Diff: Debug
        + InnerSpace
        + Neg<Output = <P::Point as EuclideanSpace>::Diff>,
    P::Aabb: Discrete<P::Aabb> + Aabb<Scalar = Real>,
    S: SimplexProcessor<Point = P::Point> + Send,
    E: EPA<Point = P::Point> + Send,
    T: Transform<P::Point>,
{
    fn collide(
        &self,
        left: &CollisionShape<P, T>,
        left_transform: &T,
        right: &CollisionShape<P, T>,
        right_transform: &T,
    ) -> Option<Contact<P::Point>> {
        if !left.enabled || !right.enabled || left.primitives.is_empty()
            || right.primitives.is_empty()
        {
            return None;
        }

        let strategy = max(&left.strategy, &right.strategy);
        self.intersection_complex(
            &strategy,
            &left.primitives,
            left_transform,
            &right.primitives,
            right_transform,
        )
    }
}

fn max(left: &CollisionStrategy, right: &CollisionStrategy) -> CollisionStrategy {
    if left > right {
        left.clone()
    } else {
        right.clone()
    }
}
