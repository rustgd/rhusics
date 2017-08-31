use collision::Aabb2;

use cgmath::{Vector2, Basis2, Point2};

use collide::{CollisionPrimitive, CollisionShape};
use collide::primitive2d::Primitive2D;
use collide::broad::BroadCollisionInfo;
use collide::broad::brute_force::BruteForce;
use collide::broad::sweep_prune::SweepAndPrune;
use collide::broad::sweep_prune::variance::Variance2D;

use collide::narrow::gjk::GJK;
use collide::narrow::gjk::simplex::SimplexProcessor2D;

use collide::ecs::resources::Contacts;
use collide::ecs::system::CollisionSystem;

pub use collide::CollisionStrategy;
pub use collide::primitive2d::*;

use BodyPose;

pub type Contacts2D<S> = Contacts<Vector2<S>>;
pub type CollisionPrimitive2D<S> = CollisionPrimitive<Primitive2D<S>, Aabb2<S>, Basis2<S>>;

pub type CollisionShape2D<S> = CollisionShape<Primitive2D<S>, Aabb2<S>, Basis2<S>>;

pub type BroadCollisionInfo2D<ID, S> = BroadCollisionInfo<ID, Aabb2<S>>;

pub type BroadBruteForce2D = BruteForce;
pub type SweepAndPrune2D<S> = SweepAndPrune<S, Variance2D<S>>;

pub type GJK2D<S> = GJK<Vector2<S>, SimplexProcessor2D>;

pub type CollisionSystem2D<S> = CollisionSystem<Primitive2D<S>, Aabb2<S>, Basis2<S>>;
pub type BodyPose2D<S> = BodyPose<Point2<S>, Basis2<S>>;
