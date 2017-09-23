//! Type wrappers and convenience functions for 3D collision detection

pub use collide::CollisionStrategy;
pub use collide::broad::SweepAndPrune3;
pub use collide::narrow::GJK3;
pub use collide::primitives::primitive3d::*;

use std::fmt::Debug;

use cgmath::{Point3, Quaternion};
use collision::dbvt::DynamicBoundingVolumeTree;
use specs::{Component, Entity, World};

use {BodyPose, Pose, Real};
use collide::*;
use collide::broad::BruteForce;
use collide::ecs::{BasicCollisionSystem, Contacts, SpatialCollisionSystem, SpatialSortingSystem};

/// Contacts resource for 3D, see [Contacts](../collide/ecs/struct.Contacts.html) for more
/// information.
pub type Contacts3 = Contacts<Point3<Real>>;

/// Collision shape for 3D, see [CollisionShape](../collide/struct.CollisionShape.html) for more
/// information
pub type CollisionShape3<T> = CollisionShape<Primitive3, T>;

/// Broad phase brute force algorithm for 3D, see
/// [BruteForce](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce3 = BruteForce;

/// ECS collision system for 3D, see
/// [BasicCollisionSystem](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
pub type BasicCollisionSystem3<T> = BasicCollisionSystem<
    Primitive3,
    T,
    ContainerShapeWrapper<Entity, Primitive3>,
>;

/// Spatial sorting system for 3D, see
/// [SpatialSortingSystem](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
pub type SpatialSortingSystem3<T> = SpatialSortingSystem<Primitive3, T>;

/// Spatial collision system for 3D, see
/// [SpatialCollisionSystem](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
pub type SpatialCollisionSystem3<T> = SpatialCollisionSystem<
    Primitive3,
    T,
    (usize,
     ContainerShapeWrapper<
        Entity,
        Primitive3,
    >),
>;

/// Body pose transform for 3D, see [BodyPose](../struct.BodyPose.html) for more information.
pub type BodyPose3 = BodyPose<Point3<Real>, Quaternion<Real>>;

/// Dynamic bounding volume tree for 3D
pub type DynamicBoundingVolumeTree3 = DynamicBoundingVolumeTree<
    ContainerShapeWrapper<
        Entity,
        Primitive3,
    >,
>;

/// Utility method for registering 3D components and resources with
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
pub fn world_register<'a, T>(world: &mut World)
where
    T: Pose<Point3<Real>> + Component + Send + Sync + 'static,
{
    world.register::<T>();
    world.register::<CollisionShape3<T>>();
    world.add_resource(Contacts3::default());
}

/// Utility method for registering 3D components and resources with
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
    T: Pose<Point3<Real>> + Component + Clone + Debug + Send + Sync + 'static,
{
    world_register::<T>(&mut world);
    world.add_resource(DynamicBoundingVolumeTree3::new());
}
