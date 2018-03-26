//! Type wrappers and convenience functions for 3D collision detection

pub use collision::algorithm::minkowski::GJK3;
pub use collision::primitive::{ConvexPolyhedron, Cuboid, Particle3, Sphere};

pub use core::{CollisionMode, CollisionStrategy};
pub use core::collide3d::*;

use cgmath::{BaseFloat, Point3, Transform};
use collision::Aabb3;
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValueWrapped};
use collision::primitive::Primitive3;
use specs::{Component, Entity, World};

use collide::{BasicCollisionSystem, SpatialCollisionSystem, SpatialSortingSystem};
use core::{Collider, ContactEvent};
use resources::WithRhusics;

/// Contact event for 2D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type ContactEvent3<S> = ContactEvent<Entity, Point3<S>>;

/// ECS collision system for 3D, see
/// [`BasicCollisionSystem`](../collide/ecs/struct.BasicCollisionSystem.html) for more information.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
/// - `Y`: Collider type, see `Collider` for more information
pub type BasicCollisionSystem3<S, T, Y = ()> =
    BasicCollisionSystem<Primitive3<S>, T, TreeValueWrapped<Entity, Aabb3<S>>, Aabb3<S>, Y>;

/// Spatial sorting system for 3D, see
/// [`SpatialSortingSystem`](../collide/ecs/struct.SpatialSortingSystem.html) for more information.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
/// - `Y`: Collider type, see `Collider` for more information
pub type SpatialSortingSystem3<S, T, Y = ()> =
    SpatialSortingSystem<Primitive3<S>, T, TreeValueWrapped<Entity, Aabb3<S>>, Aabb3<S>, Y>;

/// Spatial collision system for 3D, see
/// [`SpatialCollisionSystem`](../collide/ecs/struct.SpatialCollisionSystem.html) for more
/// information.
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
/// - `T`: Transform
/// - `Y`: Collider type, see `Collider` for more information
pub type SpatialCollisionSystem3<S, T, Y = ()> = SpatialCollisionSystem<
    Primitive3<S>,
    T,
    (usize, TreeValueWrapped<Entity, Aabb3<S>>),
    Aabb3<S>,
    Y,
>;

/// Dynamic bounding volume tree for 3D
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type DynamicBoundingVolumeTree3<S> =
    DynamicBoundingVolumeTree<TreeValueWrapped<Entity, Aabb3<S>>>;

/// Utility method for registering 3D collision components and resources with
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
pub fn register_collision<S, T, Y>(world: &mut World)
where
    S: BaseFloat + Send + Sync + 'static,
    T: Transform<Point3<S>> + Component + Send + Sync + 'static,
    Y: Collider + Send + Sync + 'static,
{
    world.register_collision::<Primitive3<S>, Aabb3<S>, T, TreeValueWrapped<Entity, Aabb3<S>>, Y>();
}
