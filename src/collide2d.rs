//! Type wrappers and convenience functions for 2D collision detection

pub use collide::CollisionStrategy;
pub use collide::primitive2d::*;

use cgmath::{Vector2, Basis2, Point2};
use collision::Aabb2;
use specs::{World, Component, Entity};

use {BodyPose, Real, Pose};
use collide::{CollisionPrimitive, CollisionShape};
use collide::broad::{BroadCollisionInfo, BruteForce, SweepAndPrune, Variance2};
use collide::dbvt::DynamicBoundingVolumeTree;
use collide::ecs::{Contacts, BasicCollisionSystem, SpatialSortingSystem, SpatialCollisionSystem};
use collide::narrow::{GJK, EPA2, SimplexProcessor2};
use collide::primitive2d::Primitive2;

/// Contacts resource for 2D, see [Contacts](../collide/ecs/struct.Contacts.html) for more
/// information.
pub type Contacts2 = Contacts<Vector2<Real>>;

/// Collision primitive for 2D, see [CollisionPrimitive](../collide/struct.CollisionPrimitive.html)
/// for more information.
pub type CollisionPrimitive2<T> = CollisionPrimitive<Primitive2, T>;

/// Collision shape for 2D, see [CollisionShape](../collide/struct.CollisionShape.html) for more
/// information
pub type CollisionShape2<T> = CollisionShape<Primitive2, T>;

/// Broad phase collision information for 2D, see
/// [BroadCollisionInfo](../collide/broad/struct.BroadCollisionInfo.html) for more information.
pub type BroadCollisionInfo2<ID> = BroadCollisionInfo<ID, Aabb2<Real>>;

/// Broad phase brute force algorithm for 2D, see
/// [BruteForce](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce2 = BruteForce;

/// Broad phase sweep and prune algorithm for 2D, see
/// [SweepAndPrune](../collide/broad/struct.SweepAndPrune.html) for more information.
pub type SweepAndPrune2 = SweepAndPrune<Variance2>;

/// GJK algorithm for 2D, see [GJK](../collide/narrow/struct.GJK.html) for more information.
pub type GJK2<T> = GJK<Point2<Real>, T, SimplexProcessor2, EPA2>;

/// Basic collision system for 2D, see
/// [BasicCollisionSystem](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
pub type BasicCollisionSystem2<T> = BasicCollisionSystem<Primitive2, T>;

/// Spatial sorting system for 2D, see
/// [SpatialSortingSystem](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
pub type SpatialSortingSystem2<T> = SpatialSortingSystem<Primitive2, T>;

/// Spatial collision system for 2D, see
/// [SpatialCollisionSystem](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
pub type SpatialCollisionSystem2<T> = SpatialCollisionSystem<Primitive2, T>;

/// Body pose transform for 2D, see [BodyPose](../struct.BodyPose.html) for more information.
pub type BodyPose2 = BodyPose<Point2<Real>, Basis2<Real>>;

/// Dynamic bounding volume tree for 2D
pub type DynamicBoundingVolumeTree2<ID> = DynamicBoundingVolumeTree<BroadCollisionInfo2<ID>>;

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
    world.register::<CollisionShape2<T>>();
    world.add_resource(Contacts2::default());
}

/// Utility method for registering 2D components and resources with
/// [`specs::World`](https://docs.rs/specs/0.9.5/specs/struct.World.html).
///
/// Will include components and resources needed for spatial sorting/collision detection.
/// Will call [`world_register`](fn.world_register.html).
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
pub fn world_register_with_spatial<T>(mut world: &mut World)
where
    T: Pose<Point2<Real>> + Component + Send + Sync + 'static,
{
    world_register::<T>(&mut world);
    world.add_resource(DynamicBoundingVolumeTree2::<Entity>::new());
}
