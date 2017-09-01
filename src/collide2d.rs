pub use collide::CollisionStrategy;
pub use collide::primitive2d::*;

use cgmath::{Vector2, Basis2, Point2};
use collision::Aabb2;
use specs::{World, Component};

use {BodyPose, Real, Pose};
use collide::{CollisionPrimitive, CollisionShape};
use collide::broad::BroadCollisionInfo;
use collide::broad::brute_force::BruteForce;
use collide::broad::sweep_prune::SweepAndPrune;
use collide::broad::sweep_prune::variance::Variance2D;
use collide::ecs::resources::Contacts;
use collide::ecs::system::CollisionSystem;
use collide::narrow::gjk::GJK;
use collide::narrow::gjk::simplex::SimplexProcessor2D;
use collide::primitive2d::Primitive2D;

pub type Contacts2D = Contacts<Vector2<Real>>;
pub type CollisionPrimitive2D<T> = CollisionPrimitive<Primitive2D, Aabb2<Real>, T>;

pub type CollisionShape2D<T> = CollisionShape<Primitive2D, Aabb2<Real>, T>;

pub type BroadCollisionInfo2D<ID> = BroadCollisionInfo<ID, Aabb2<Real>>;

pub type BroadBruteForce2D = BruteForce;
pub type SweepAndPrune2D = SweepAndPrune<Variance2D>;

pub type GJK2D = GJK<Vector2<Real>, SimplexProcessor2D>;

pub type CollisionSystem2D<T> = CollisionSystem<Primitive2D, Aabb2<Real>, T>;
pub type BodyPose2D = BodyPose<Point2<Real>, Basis2<Real>>;

pub fn world_register<'a, T>(world: &mut World)
where
    T : Pose<Point2<Real>> + Component + Send + Sync + 'static,
{
    world.register::<T>();
    world.register::<CollisionShape2D<T>>();
    world.add_resource(Contacts2D::new());
}
