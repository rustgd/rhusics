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

use Real;
use collide::{CollisionShape, Primitive};
use collide::dbvt::TreeValue;

mod sweep_prune;
mod brute_force;

/// Collision info data that the broad phase algorithms work with.
///
/// # Type parameters:
///
/// - `ID`: id type of collision shapes
/// - `A`: Aabb bounding box type
#[derive(Debug, Clone)]
pub struct BroadCollisionInfo<ID, A> {
    id: ID,
    bound: A,
    index: usize,
}

impl<ID, A> BroadCollisionInfo<ID, A> {
    /// Create a new collision info
    pub fn new(id: ID, bound: A) -> Self {
        Self {
            id,
            bound,
            index: 0,
        }
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

impl<ID, A> TreeValue for BroadCollisionInfo<ID, A>
where
    ID: Clone + Debug,
    A: Aabb<Scalar = Real> + Clone + Debug,
    A::Diff: VectorSpace<Scalar=Real>,
{
    type Bound = A;
    type Vector = A::Diff;

    fn bound(&self) -> &A {
        &self.bound
    }

    fn fat_bound(&self) -> A {
        self.bound.clone()
    }

    fn set_index(&mut self, index: usize) {
        self.index = index;
    }

    fn index(&self) -> usize {
        self.index
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
