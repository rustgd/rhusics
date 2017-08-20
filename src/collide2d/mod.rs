use collide::{Contact, CollisionPrimitive, CollisionShape};

use collide::broad::BroadCollisionInfo;
use collide::broad::brute_force::BruteForce;
use collide::broad::sweep_prune::SweepAndPrune;

use collide::narrow::gjk::GJK;
use collide::narrow::gjk::simplex::SimplexProcessor2D;

use cgmath::{Vector2, Point2, Basis2, BaseFloat};
use collision::Aabb2;

use specs::{Component, DenseVecStorage};

use std::fmt::Debug;

pub use collide::CollisionStrategy;
pub use collide::primitive2d::*;

pub type Contact2D<ID, S> = Contact<ID, S, Vector2<S>>;
pub type CollisionPrimitive2D<S> = CollisionPrimitive<S, Vector2<S>, Point2<S>, Basis2<S>, Aabb2<S>>;
pub type CollisionShape2D<ID, S> = CollisionShape<ID, S, Vector2<S>, Point2<S>, Basis2<S>, Aabb2<S>>;

pub type BroadCollisionInfo2D<ID, S> = BroadCollisionInfo<ID, S, Vector2<S>, Point2<S>, Aabb2<S>>;
pub type BroadBruteForce2D = BruteForce;
pub type SweepAndPrune2D = SweepAndPrune;

pub type GJK2D<S> = GJK<S, Vector2<S>, SimplexProcessor2D>;

impl <ID, S> Component for CollisionShape2D<ID, S>
    where
        ID : Clone + Send + Sync + Debug + 'static,
        S : BaseFloat + Send + Sync + 'static,
{
    type Storage = DenseVecStorage<CollisionShape2D<ID, S>>;
}