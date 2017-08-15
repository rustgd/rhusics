pub use two::collision::CollisionShape;
use collision::Aabb2;

pub mod sweep_prune;

use std::fmt::Debug;
use std::clone::Clone;

#[derive(Debug)]
pub struct BroadCollisionInfo<ID: Debug + Clone> {
    id: ID,
    bound: Aabb2<f32>,
}
pub trait BroadPhase<ID: Debug + Clone> {
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID>>) -> Vec<(ID, ID)>;
}
