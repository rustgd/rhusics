//! Type wrappers and convenience functions for 2D collision detection

pub use collide::CollisionStrategy;
pub use collide::primitive2d::*;

use cgmath::{Vector2, Basis2, Point2};
use collision::Aabb2;
use specs::{World, Component};

use {BodyPose, Real, Pose};
use collide::{CollisionPrimitive, CollisionShape};
use collide::broad::{BroadCollisionInfo, BruteForce, SweepAndPrune, Variance2D};
use collide::ecs::{Contacts, CollisionSystem};
use collide::narrow::{GJK, EPA2D, SimplexProcessor2D};
use collide::primitive2d::Primitive2D;

/// Contacts resource for 2D, see [Contacts](../collide/ecs/struct.Contacts.html) for more
/// information.
pub type Contacts2D = Contacts<Vector2<Real>>;

/// Collision primitive for 2D, see [CollisionPrimitive](../collide/struct.CollisionPrimitive.html)
/// for more information.
pub type CollisionPrimitive2D<T> = CollisionPrimitive<Primitive2D, T>;

/// Collision shape for 2D, see [CollisionShape](../collide/struct.CollisionShape.html) for more
/// information
pub type CollisionShape2D<T> = CollisionShape<Primitive2D, T>;

/// Broad phase collision information for 2D, see
/// [BroadCollisionInfo](../collide/broad/struct.BroadCollisionInfo.html) for more information.
pub type BroadCollisionInfo2D<ID> = BroadCollisionInfo<ID, Aabb2<Real>>;

/// Broad phase brute force algorithm for 2D, see
/// [BruteForce](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce2D = BruteForce;

/// Broad phase sweep and prune algorithm for 2D, see
/// [SweepAndPrune](../collide/broad/struct.SweepAndPrune.html) for more information.
pub type SweepAndPrune2D = SweepAndPrune<Variance2D>;

/// GJK algorithm for 2D, see [GJK](../collide/narrow/struct.GJK.html) for more information.
pub type GJK2D<T> = GJK<Point2<Real>, T, SimplexProcessor2D, EPA2D>;

/// ECS collision system for 2D, see [CollisionSystem](../collide/ecs/struct.CollisionSystem.html)
/// for more information.
pub type CollisionSystem2D<T> = CollisionSystem<Primitive2D, T>;

/// Body pose transform for 2D, see [BodyPose](../struct.BodyPose.html) for more information.
pub type BodyPose2D = BodyPose<Point2<Real>, Basis2<Real>>;

/// Utility method for registering 2D components and resources with
/// [`specs::World`](https://docs.rs/specs/0.9.5/specs/struct.World.html).
///
/// # Parameters
///
/// - `world`: The [world](https://docs.rs/specs/0.9.5/specs/struct.World.html)
/// to register components/resources in.
///
/// # Type parameters
///
/// - `T`: Transform type that implements [`Pose`](../trait.Pose.html) and
///        [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html).
pub fn world_register<T>(world: &mut World)
where
    T: Pose<Point2<Real>> + Component + Send + Sync + 'static,
{
    world.register::<T>();
    world.register::<CollisionShape2D<T>>();
    world.add_resource(Contacts2D::default());
}
