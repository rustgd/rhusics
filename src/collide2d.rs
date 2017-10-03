//! Type wrappers and convenience functions for 2D collision detection

pub use collide::CollisionStrategy;
pub use collision::algorithm::minkowski::GJK2;
pub use collision::primitive::{Circle, ConvexPolygon, Particle2, Rectangle};

use std::fmt::Debug;

use cgmath::{Basis2, Point2};
use collision::algorithm::broad_phase::BruteForce;
use collision::dbvt::DynamicBoundingVolumeTree;
use collision::primitive::Primitive2;
use specs::{Component, Entity, World};

use {BodyPose, Pose, Real};
use collide::*;
use collide::ecs::{BasicCollisionSystem, Contacts, SpatialCollisionSystem, SpatialSortingSystem};

/// Contacts resource for 2D, see [Contacts](../collide/ecs/struct.Contacts.html) for more
/// information.
pub type Contacts2 = Contacts<Point2<Real>>;

/// Collision shape for 2D, see [CollisionShape](../collide/struct.CollisionShape.html) for more
/// information
pub type CollisionShape2<T> = CollisionShape<Primitive2<Real>, T>;

/// Broad phase brute force algorithm for 2D, see
/// [BruteForce](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce2 = BruteForce;

/// Broad phase sweep and prune algorithm
pub type SweepAndPrune2 = ::collision::algorithm::broad_phase::SweepAndPrune2<Real>;

/// Basic collision system for 2D, see
/// [BasicCollisionSystem](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
pub type BasicCollisionSystem2<T> = BasicCollisionSystem<
    Primitive2<Real>,
    T,
    ContainerShapeWrapper<Entity, Primitive2<Real>>,
>;

/// Spatial sorting system for 2D, see
/// [SpatialSortingSystem](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
pub type SpatialSortingSystem2<T> = SpatialSortingSystem<Primitive2<Real>, T>;

/// Spatial collision system for 2D, see
/// [SpatialCollisionSystem](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
pub type SpatialCollisionSystem2<T> = SpatialCollisionSystem<
    Primitive2<Real>,
    T,
    (usize, ContainerShapeWrapper<Entity, Primitive2<Real>>),
>;

/// Body pose transform for 2D, see [BodyPose](../struct.BodyPose.html) for more information.
pub type BodyPose2 = BodyPose<Point2<Real>, Basis2<Real>>;

/// Dynamic bounding volume tree for 2D

pub type DynamicBoundingVolumeTree2 = DynamicBoundingVolumeTree<
    ContainerShapeWrapper<Entity, Primitive2<Real>>,
>;

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
    T: Pose<Point2<Real>> + Component + Clone + Debug + Send + Sync + 'static,
{
    world_register::<T>(&mut world);
    world.add_resource(DynamicBoundingVolumeTree2::new());
}
