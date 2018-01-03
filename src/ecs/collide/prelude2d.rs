//! Type wrappers and convenience functions for 2D collision detection

pub use collision::algorithm::minkowski::GJK2;
pub use collision::primitive::{Circle, ConvexPolygon, Particle2, Rectangle};

pub use collide::{CollisionMode, CollisionStrategy};
pub use collide::prelude2d::*;

use cgmath::{BaseFloat, Point2, Transform};
use collision::Aabb2;
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValueWrapped};
use collision::primitive::Primitive2;
use specs::{Component, Entity, World};

use collide::ContactEvent;
use ecs::WithRhusics;
use ecs::collide::{BasicCollisionSystem, SpatialCollisionSystem, SpatialSortingSystem};

/// Contact event for 2D
pub type ContactEvent2<S> = ContactEvent<Entity, Point2<S>>;

/// Basic collision system for 2D, see
/// [BasicCollisionSystem](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
pub type BasicCollisionSystem2<S, T, Y = ()> = BasicCollisionSystem<
    Primitive2<S>,
    T,
    TreeValueWrapped<Entity, Aabb2<S>>,
    Aabb2<S>,
    Y,
>;

/// Spatial sorting system for 2D, see
/// [SpatialSortingSystem](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
pub type SpatialSortingSystem2<S, T, Y = ()> = SpatialSortingSystem<
    Primitive2<S>,
    T,
    TreeValueWrapped<Entity, Aabb2<S>>,
    Aabb2<S>,
    Y,
>;

/// Spatial collision system for 2D, see
/// [SpatialCollisionSystem](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
pub type SpatialCollisionSystem2<S, T, Y = ()> = SpatialCollisionSystem<
    Primitive2<S>,
    T,
    (usize, TreeValueWrapped<Entity, Aabb2<S>>),
    Aabb2<S>,
    Y,
>;

/// Dynamic bounding volume tree for 2D

pub type DynamicBoundingVolumeTree2<S> = DynamicBoundingVolumeTree<
    TreeValueWrapped<Entity, Aabb2<S>>,
>;

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
pub fn register_collision<S, T, Y>(world: &mut World)
where
    S: BaseFloat + Send + Sync + 'static,
    T: Transform<Point2<S>> + Component + Send + Sync + 'static,
    Y: Collider + Send + Sync + 'static,
{
    world.register_collision::<Primitive2<S>, Aabb2<S>, T, TreeValueWrapped<Entity, Aabb2<S>>, Y>();
}
