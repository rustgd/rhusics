use collision::{Aabb, Discrete};
use cgmath::prelude::*;
use cgmath::BaseFloat;
use collide::CollisionShape;

pub mod sweep_prune;
pub mod brute_force;

use std::fmt::Debug;
use std::clone::Clone;
use std;

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

impl<ID, P, A, R> From<(ID, CollisionShape<P, A, R>)> for BroadCollisionInfo<ID, A>
where
    A: Aabb + Clone,
    A::Diff: Debug,
{
    fn from((id, shape): (ID, CollisionShape<P, A, R>)) -> Self {
        Self::new(id, shape.transformed_bound.clone())
    }
}

pub trait BroadPhase<ID, A> {
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID, A>>) -> Vec<(ID, ID)>;
}
