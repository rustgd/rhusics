//! Type wrappers and convenience functions for 2D collision detection

pub use collision::algorithm::minkowski::GJK2;
pub use collision::primitive::{Circle, ConvexPolygon, Particle2, Rectangle};

pub use collide::{CollisionMode, CollisionStrategy};
pub use collide::prelude2d::*;

use cgmath::{Point2, Transform};
use collision::Aabb2;
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValueWrapped};
use collision::primitive::Primitive2;
use specs::{Component, Entity, World};

use Real;
use collide::ContactEvent;
use ecs::WithRhusics;
use ecs::collide::{BasicCollisionSystem, SpatialCollisionSystem, SpatialSortingSystem};

/// Contact event for 2D
pub type ContactEvent2 = ContactEvent<Entity, Point2<Real>>;

/// Basic collision system for 2D, see
/// [BasicCollisionSystem](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
pub type BasicCollisionSystem2<T, Y = ()> = BasicCollisionSystem<
    Primitive2<Real>,
    T,
    TreeValueWrapped<Entity, Aabb2<Real>>,
    Aabb2<Real>,
    Y,
>;

/// Spatial sorting system for 2D, see
/// [SpatialSortingSystem](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
pub type SpatialSortingSystem2<T, Y = ()> = SpatialSortingSystem<
    Primitive2<Real>,
    T,
    TreeValueWrapped<Entity, Aabb2<Real>>,
    Aabb2<Real>,
    Y,
>;

/// Spatial collision system for 2D, see
/// [SpatialCollisionSystem](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
pub type SpatialCollisionSystem2<T, Y = ()> = SpatialCollisionSystem<
    Primitive2<Real>,
    T,
    (usize, TreeValueWrapped<Entity, Aabb2<Real>>),
    Aabb2<Real>,
    Y,
>;

/// Dynamic bounding volume tree for 2D

pub type DynamicBoundingVolumeTree2 =
    DynamicBoundingVolumeTree<TreeValueWrapped<Entity, Aabb2<Real>>>;

/// Utility method for registering 2D collision components and resources with
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
/// - `Y`: Shape type, see `Collider`
pub fn register_collision<T, Y>(world: &mut World)
where
    T: Transform<Point2<Real>> + Component + Send + Sync + 'static,
    Y: Collider + Send + Sync + 'static,
{
    world.register_collision::<
        Primitive2<Real>,
        Aabb2<Real>,
        T,
        TreeValueWrapped<Entity, Aabb2<Real>>,
        Y
    >();
}
