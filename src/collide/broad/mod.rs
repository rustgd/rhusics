//! Generic broad phase collision detection algorithms
//!
//! Currently supports the following algorithms:
//!
//! - `BruteForce`: compares all bounding boxes. O(n^2 ).
//! - `SweepAndPrune`: will sort bounding boxes along one of the axis, and do overlap tests.
//!                    Best case O(n), worst case O(n^2 ).
//!

pub use self::brute_force::BruteForce;
pub use self::sweep_prune::{SweepAndPrune, Variance2D, Variance3D};

use std::clone::Clone;
use std::fmt::Debug;

use cgmath::prelude::*;
use collision::{Aabb, Discrete};

use collide::{CollisionShape, Primitive};

mod sweep_prune;
mod brute_force;

/// Collision info data that the broad phase algorithms work with.
///
/// # Type parameters:
///
/// - `ID`: id type of collision shapes
/// - `A`: Aabb bounding box type
#[derive(Debug)]
pub struct BroadCollisionInfo<ID, A> {
    id: ID,
    bound: A,
}

impl<ID, A> BroadCollisionInfo<ID, A> {
    /// Create a new collision info
    pub fn new(id: ID, bound: A) -> Self {
        Self { id, bound }
    }
}

impl<ID, P, T> From<(ID, CollisionShape<P, T>)> for BroadCollisionInfo<ID, P::Aabb>
where
    P: Primitive,
{
    fn from((id, shape): (ID, CollisionShape<P, T>)) -> Self {
        Self::new(id, shape.transformed_bound.clone())
    }
}

/// Trait implemented by all broad phase algorithms.
///
/// # Type parameters:
///
/// - `ID`: id type of collision shapes
/// - `A`: Aabb bounding box type
///
pub trait BroadPhase<ID, A>: Debug {
    /// Compute a list of potentially colliding shapes.
    ///
    /// # Parameters:
    ///
    /// - `shapes`: list with collision information about each shape in the collision world
    ///
    /// # Returns
    ///
    /// Returns a list of potentially colliding shape pairs
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID, A>>) -> Vec<(ID, ID)>;
}
