//! Generic broad phase collision detection algorithms
//!
//! Currently supports the following algorithms:
//!
//! - `BruteForce`: compares all bounding boxes. O(n^2 ).
//! - `SweepAndPrune`: will sort bounding boxes along one of the axis, and do overlap tests.
//!                    Best case O(n), worst case O(n^2 ).
//!

pub use self::brute_force::BruteForce;
pub use self::sweep_prune::{SweepAndPrune, SweepAndPrune2, SweepAndPrune3};

use std::fmt::Debug;

use cgmath::prelude::*;
use collision::{Aabb, Discrete};

mod sweep_prune;
mod brute_force;

/// Trait used by values for broad phase
pub trait BroadCollisionData {
    /// Id type
    type Id;

    /// Bounding volume type
    type Bound;

    /// Return the id of the shape
    fn id(&self) -> &Self::Id;

    /// Return the bounding volume of the shape
    fn bound(&self) -> &Self::Bound;
}

/// Trait implemented by all broad phase algorithms.
///
/// # Type parameters:
///
/// - `ID`: id type of collision shapes
/// - `A`: Aabb bounding box type
///
pub trait BroadPhase<D>: Debug + Send
where
    D: BroadCollisionData,
{
    /// Compute a list of potentially colliding shapes.
    ///
    /// # Parameters:
    ///
    /// - `shapes`: list with collision information about each shape in the collision world
    ///
    /// # Returns
    ///
    /// Returns a list of potentially colliding shape pairs
    fn compute(&mut self, shapes: &mut Vec<D>) -> Vec<(D::Id, D::Id)>;
}
