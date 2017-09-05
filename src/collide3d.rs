//! Type wrappers and convenience functions for 3D collision detection

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

/// Contacts resource for 3D, see [Contacts](../collide/ecs/struct.Contacts.html) for more
/// information.
pub type Contacts3D = Contacts<Vector3<Real>>;

/// Collision primitive for 3D, see [CollisionPrimitive](../collide/struct.CollisionPrimitive.html)
/// for more information.
pub type CollisionPrimitive3D<T> = CollisionPrimitive<Primitive3D, T>;

/// Collision shape for 3D, see [CollisionShape](../collide/struct.CollisionShape.html) for more
/// information
pub type CollisionShape3D<T> = CollisionShape<Primitive3D, T>;

/// Broad phase collision information for 3D, see
/// [BroadCollisionInfo](../collide/broad/struct.BroadCollisionInfo.html) for more information.
pub type BroadCollisionInfo3D<ID> = BroadCollisionInfo<ID, Aabb3<Real>>;

/// Broad phase brute force algorithm for 3D, see
/// [BruteForce](../collide/broad/struct.BruteForce.html) for more information.
pub type BroadBruteForce3D = BruteForce;

/// Broad phase sweep and prune algorithm for 3D, see
/// [SweepAndPrune](../collide/broad/struct.SweepAndPrune.html) for more information.
pub type SweepAndPrune3D = SweepAndPrune<Variance3D>;

/// GJK algorithm for 3D, see [GJK](../collide/narrow/struct.GJK.html) for more information.
pub type GJK3D<T> = GJK<Point3<Real>, T, SimplexProcessor3D, EPA3D>;

/// ECS collision system for 3D, see [CollisionSystem](../collide/ecs/struct.CollisionSystem.html)
/// for more information.
pub type CollisionSystem3D<T> = CollisionSystem<Primitive3D, T>;

/// Body pose transform for 3D, see [BodyPose](../struct.BodyPose.html) for more information.
pub type BodyPose3D = BodyPose<Point3<Real>, Quaternion<Real>>;

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
    world.register::<CollisionShape3D<T>>();
    world.add_resource(Contacts3D::default());
}
