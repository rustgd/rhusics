pub use collide::CollisionStrategy;
pub use collide::primitive3d::*;

use cgmath::{Vector3, Quaternion, Point3};
use collision::Aabb3;
use specs::{World, Component};

use {BodyPose, Real, Pose};
use collide::{CollisionPrimitive, CollisionShape};
use collide::broad::{BroadCollisionInfo, BruteForce, SweepAndPrune, Variance3D};
use collide::ecs::{Contacts, CollisionSystem};
use collide::narrow::{GJK, EPA3D, SimplexProcessor3D};

pub type Contacts3D = Contacts<Vector3<Real>>;
pub type CollisionPrimitive3D<T> = CollisionPrimitive<Primitive3D, Aabb3<Real>, T>;

pub type CollisionShape3D<T> = CollisionShape<Primitive3D, Aabb3<Real>, T>;

pub type BroadCollisionInfo3D<ID> = BroadCollisionInfo<ID, Aabb3<Real>>;

pub type BroadBruteForce3D = BruteForce;
pub type SweepAndPrune3D = SweepAndPrune<Variance3D>;

pub type GJK3D<T> = GJK<Point3<Real>, T, SimplexProcessor3D, EPA3D>;

pub type CollisionSystem3D<T> = CollisionSystem<Primitive3D, Aabb3<Real>, T>;
pub type BodyPose3D = BodyPose<Point3<Real>, Quaternion<Real>>;

pub fn world_register<'a, T>(world: &mut World)
where
    T: Pose<Point3<Real>> + Component + Send + Sync + 'static,
{
    world.register::<T>();
    world.register::<CollisionShape3D<T>>();
    world.add_resource(Contacts3D::new());
}
