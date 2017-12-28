//! Type wrappers and convenience functions for 3D collision detection

pub use collision::algorithm::minkowski::GJK3;
pub use collision::primitive::{ConvexPolyhedron, Cuboid, Particle3, Sphere};

pub use collide::{CollisionMode, CollisionStrategy};
pub use collide::prelude3d::*;

use std::fmt::Debug;

use cgmath::{Point3, Transform};
use collision::dbvt::DynamicBoundingVolumeTree;
use collision::primitive::Primitive3;
use specs::{Component, Entity, World};
use shrev::EventChannel;

use {NextFrame, Real};
use collide::ContactEvent;
use collide::util::ContainerShapeWrapper;
use ecs::collide::{BasicCollisionSystem, SpatialCollisionSystem, SpatialSortingSystem};

/// Contact event for 2D
pub type ContactEvent3 = ContactEvent<Entity, Point3<Real>>;

/// ECS collision system for 3D, see
/// [BasicCollisionSystem](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
pub type BasicCollisionSystem3<T, Y = ()> = BasicCollisionSystem<
    Primitive3<Real>,
    T,
    ContainerShapeWrapper<Entity, Primitive3<Real>>,
    Y,
>;

/// Spatial sorting system for 3D, see
/// [SpatialSortingSystem](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
pub type SpatialSortingSystem3<T, Y = ()> = SpatialSortingSystem<
    Primitive3<Real>,
    T,
    ContainerShapeWrapper<Entity, Primitive3<Real>>,
    Y,
>;

/// Spatial collision system for 3D, see
/// [SpatialCollisionSystem](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
pub type SpatialCollisionSystem3<T, Y = ()> = SpatialCollisionSystem<
    Primitive3<Real>,
    T,
    (usize, ContainerShapeWrapper<Entity, Primitive3<Real>>),
    Y,
>;

/// Dynamic bounding volume tree for 3D
pub type DynamicBoundingVolumeTree3 = DynamicBoundingVolumeTree<
    ContainerShapeWrapper<Entity, Primitive3<Real>>,
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
/// - `Y`: Shape type, see `Collider`
pub fn world_register<'a, T, Y>(world: &mut World)
where
    T: Transform<Point3<Real>> + Component + Send + Sync + 'static,
    Y: Send + Sync + 'static,
{
    world.register::<T>();
    world.register::<NextFrame<T>>();
    world.register::<CollisionShape3<T, Y>>();
    world.add_resource(EventChannel::<ContactEvent3>::new());
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
/// - `Y`: Shape type, see `Collider`
pub fn world_register_with_spatial<T, Y>(mut world: &mut World)
where
    T: Transform<Point3<Real>> + Component + Clone + Debug + Send + Sync + 'static,
    Y: Send + Sync + 'static,
{
    world_register::<T, Y>(&mut world);
    world.add_resource(DynamicBoundingVolumeTree3::new());
}
