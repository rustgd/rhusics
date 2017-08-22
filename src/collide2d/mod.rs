use cgmath::{Vector2, Point2, Basis2};
use collision::Aabb2;

use collide::{CollisionPrimitive, CollisionShape};
use collide::ecs::CollisionSystem;

use collide::broad::BroadCollisionInfo;
use collide::broad::brute_force::BruteForce;
use collide::broad::sweep_prune::SweepAndPrune;

use collide::narrow::gjk::GJK;
use collide::narrow::gjk::simplex::SimplexProcessor2D;

use collide::ecs::Contacts;

pub use collide::CollisionStrategy;
pub use collide::primitive2d::*;

use BodyPose;

pub type Contacts2D<S> = Contacts<S, Vector2<S>>;
pub type CollisionPrimitive2D<S> = CollisionPrimitive<
    S,
    Vector2<S>,
    Point2<S>,
    Basis2<S>,
    Aabb2<S>,
>;
pub type CollisionShape2D<S> = CollisionShape<S, Vector2<S>, Point2<S>, Basis2<S>, Aabb2<S>>;

pub type BroadCollisionInfo2D<ID, S> = BroadCollisionInfo<ID, S, Vector2<S>, Point2<S>, Aabb2<S>>;
pub type BroadBruteForce2D = BruteForce;
pub type SweepAndPrune2D = SweepAndPrune;

pub type GJK2D<S> = GJK<S, Vector2<S>, SimplexProcessor2D>;

pub type CollisionSystem2D<S> = CollisionSystem<S, Vector2<S>, Point2<S>, Basis2<S>, Aabb2<S>>;

pub type BodyPose2D<S> = BodyPose<S, Vector2<S>, Point2<S>, Basis2<S>>;
