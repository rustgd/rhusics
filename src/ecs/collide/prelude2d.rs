//! Type wrappers and convenience functions for 2D collision detection

pub use collision::algorithm::minkowski::GJK2;
pub use collision::primitive::{Circle, ConvexPolygon, Particle2, Rectangle};

pub use collide::{CollisionMode, CollisionStrategy};
pub use collide::prelude2d::*;

use std::fmt::Debug;

use cgmath::{Point2, Transform};
use collision::dbvt::DynamicBoundingVolumeTree;
use collision::primitive::Primitive2;
use specs::{Component, Entity, World};

use {NextFrame, Real};
use collide::ContactEvent;
use collide::util::ContainerShapeWrapper;
use ecs::collide::{BasicCollisionSystem, Contacts, SpatialCollisionSystem, SpatialSortingSystem};

/// Contacts resource for 2D, see [Contacts](../collide/ecs/struct.Contacts.html) for more
/// information.
pub type Contacts2 = Contacts<Point2<Real>>;

/// Contact event for 2D
pub type ContactEvent2 = ContactEvent<Entity, Point2<Real>>;

/// Basic collision system for 2D, see
/// [BasicCollisionSystem](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
pub type BasicCollisionSystem2<T, Y> = BasicCollisionSystem<
    Primitive2<Real>,
    T,
    Y,
    ContainerShapeWrapper<Entity, Primitive2<Real>>,
>;

/// Spatial sorting system for 2D, see
/// [SpatialSortingSystem](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
pub type SpatialSortingSystem2<T, Y> = SpatialSortingSystem<
    Primitive2<Real>,
    T,
    Y,
    ContainerShapeWrapper<Entity, Primitive2<Real>>,
>;

/// Spatial collision system for 2D, see
/// [SpatialCollisionSystem](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
pub type SpatialCollisionSystem2<T, Y> = SpatialCollisionSystem<
    Primitive2<Real>,
    T,
    Y,
    (usize, ContainerShapeWrapper<Entity, Primitive2<Real>>),
>;

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
/// - `T`: Transform type that implements
///        [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html).
pub fn world_register<T, Y>(world: &mut World)
where
    T: Transform<Point2<Real>> + Component + Send + Sync + 'static,
    Y: Send + Sync + 'static,
{
    world.register::<T>();
    world.register::<NextFrame<T>>();
    world.register::<CollisionShape2<T, Y>>();
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
/// - `T`: Transform type that implements
///        [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html).
pub fn world_register_with_spatial<T, Y>(mut world: &mut World)
where
    T: Transform<Point2<Real>> + Component + Clone + Debug + Send + Sync + 'static,
    Y: Send + Sync + 'static,
{
    world_register::<T, Y>(&mut world);
    world.add_resource(DynamicBoundingVolumeTree2::new());
}
