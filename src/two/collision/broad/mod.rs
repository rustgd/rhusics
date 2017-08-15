pub use two::collision::CollisionShape;
use collision::Aabb2;
use cgmath::{Point2, Array};

pub mod sweep_prune;
pub mod brute_force;

use std::fmt::Debug;
use std::clone::Clone;

#[derive(Debug)]
pub struct BroadCollisionInfo<ID: Debug + Clone> {
    id: ID,
    bound: Aabb2<f32>,
}

impl<ID: Debug + Clone> BroadCollisionInfo<ID> {
    pub fn new(id: ID) -> Self {
        Self::new_impl(
            id,
            Aabb2::new(Point2::from_value(0.), Point2::from_value(0.)),
        )
    }

    pub fn new_impl(id: ID, bound: Aabb2<f32>) -> Self {
        Self { id, bound }
    }
}

pub trait BroadPhase<ID: Debug + Clone> {
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID>>) -> Vec<(ID, ID)>;
}
