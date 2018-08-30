//! Type wrappers and convenience functions for 2D collision detection

pub use collision::algorithm::minkowski::GJK2;
pub use collision::primitive::{Circle, ConvexPolygon, Particle2, Rectangle};

pub use core::collide2d::*;
pub use core::{CollisionMode, CollisionStrategy};

use cgmath::Point2;
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValueWrapped};
use collision::primitive::Primitive2;
use collision::Aabb2;
use specs::prelude::Entity;

use collide::{BasicCollisionSystem, SpatialCollisionSystem, SpatialSortingSystem};
use core::ContactEvent;

/// Contact event for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type ContactEvent2<S> = ContactEvent<Entity, Point2<S>>;

/// Basic collision system for 2D, see
/// [`BasicCollisionSystem`](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
/// - `Y`: Collider type, see `Collider` for more information
pub type BasicCollisionSystem2<S, T, Y = ()> =
    BasicCollisionSystem<Primitive2<S>, T, TreeValueWrapped<Entity, Aabb2<S>>, Aabb2<S>, Y>;

/// Spatial sorting system for 2D, see
/// [`SpatialSortingSystem`](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
/// - `Y`: Collider type, see `Collider` for more information
pub type SpatialSortingSystem2<S, T, Y = ()> =
    SpatialSortingSystem<Primitive2<S>, T, TreeValueWrapped<Entity, Aabb2<S>>, Aabb2<S>, Y>;

/// Spatial collision system for 2D, see
/// [`SpatialCollisionSystem`](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
/// - `Y`: Collider type, see `Collider` for more information
pub type SpatialCollisionSystem2<S, T, Y = ()> = SpatialCollisionSystem<
    Primitive2<S>,
    T,
    (usize, TreeValueWrapped<Entity, Aabb2<S>>),
    Aabb2<S>,
    Y,
>;

/// Dynamic bounding volume tree for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type DynamicBoundingVolumeTree2<S> =
    DynamicBoundingVolumeTree<TreeValueWrapped<Entity, Aabb2<S>>>;
