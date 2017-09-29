//! Generic narrow phase collision detection algorithms.
//!
//! Currently only supports GJK/EPA.

pub use self::gjk::{GJK2, GJK3, GJK};

use std::fmt::Debug;

use cgmath::EuclideanSpace;

use collide::{CollisionShape, Contact, Primitive};

mod gjk;

/// Base trait implemented by all narrow phase algorithms.
///
/// # Type parameters:
///
/// - `P`: collision primitive type
/// - `T`: model-to-world transform type
pub trait NarrowPhase<P, T>: Debug
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
