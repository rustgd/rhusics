pub use self::brute_force::BruteForce;
pub use self::sweep_prune::{SweepAndPrune, Variance2D, Variance3D};

use std::clone::Clone;
use std::fmt::Debug;

use cgmath::prelude::*;
use collision::{Aabb, Discrete};

use collide::CollisionShape;

mod sweep_prune;
mod brute_force;

#[derive(Debug)]
pub struct BroadCollisionInfo<ID, A> {
    id: ID,
    bound: A,
}

impl<ID, A> BroadCollisionInfo<ID, A> {
    pub fn new(id: ID, bound: A) -> Self {
        Self { id, bound }
    }
}

impl<ID, P, A, T> From<(ID, CollisionShape<P, A, T>)> for BroadCollisionInfo<ID, A>
where
    A: Aabb + Clone,
    A::Diff: Debug,
{
    fn from((id, shape): (ID, CollisionShape<P, A, T>)) -> Self {
        Self::new(id, shape.transformed_bound.clone())
    }
}

pub trait BroadPhase<ID, A> {
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID, A>>) -> Vec<(ID, ID)>;
}
