use std::fmt::Debug;

use collision::prelude::*;
use specs::{System, ReadStorage, Entity, FetchMut, Component};

use {Pose, Real};
use collide::{CollisionShape, CollisionStrategy, ContactSet, Primitive};
use collide::broad::{BroadPhase, BroadCollisionInfo};
use collide::dbvt::DynamicBoundingVolumeTree;
use collide::ecs::resources::Contacts;
use collide::narrow::NarrowPhase;

/// Collision detection [system](https://docs.rs/specs/0.9.5/specs/trait.System.html) for use with
/// [`specs`](https://docs.rs/specs/0.9.5/specs/).
///
/// Will perform spatial sorting of the collision world.
///
/// Has support for both broad phase and narrow phase collision detection. Will only do narrow phase
/// if both broad and narrow phase is activated.
///
/// Can handle any transform component type, as long as the type implements
/// [`Pose`](../../trait.Pose.html) and
/// [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html).
///
#[derive(Debug)]
pub struct SpatialCollisionSystem<P, T>
where
    P: Primitive,
    P::Aabb: Clone + Debug,
{
    narrow: Option<Box<NarrowPhase<Entity, P, T>>>,
    broad: Option<Box<BroadPhase<Entity, P::Aabb>>>,
}

impl<P, T> SpatialCollisionSystem<P, T>
where
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Clone
        + Debug
        + Send
        + Sync
        + 'static
        + Union<P::Aabb, Output = P::Aabb>
        + Contains<P::Aabb>
        + SurfaceArea<Real>,
    T: Pose<P::Point> + Component,
{
    /// Create a new collision detection system, with no broad or narrow phase activated.
    pub fn new() -> Self {
        Self {
            narrow: None,
            broad: None,
        }
    }

    /// Specify what narrow phase algorithm to use
    pub fn with_narrow_phase<N: NarrowPhase<Entity, P, T> + 'static>(mut self, narrow: N) -> Self {
        self.narrow = Some(Box::new(narrow));
        self
    }

    /// Specify what broad phase algorithm to use
    pub fn with_broad_phase<B: BroadPhase<Entity, P::Aabb> + 'static>(mut self, broad: B) -> Self {
        self.broad = Some(Box::new(broad));
        self
    }
}

impl<'a, P, T> System<'a> for SpatialCollisionSystem<P, T>
where
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Clone
        + Debug
        + Send
        + Sync
        + 'static
        + Contains<P::Aabb>
        + SurfaceArea<Real>,
    P::Vector: Send + Sync + 'static,
    T: Component
        + Pose<P::Point>
        + Send
        + Sync
        + 'static,
{
    type SystemData = (ReadStorage<'a, T>,
     ReadStorage<'a, CollisionShape<P, T>>,
     FetchMut<'a, Contacts<P::Vector>>,
     FetchMut<'a, DynamicBoundingVolumeTree<BroadCollisionInfo<Entity, P::Aabb>>>);

    fn run(&mut self, (poses, shapes, mut contacts, mut tree): Self::SystemData) {
        contacts.clear();

        if let Some(ref mut broad) = self.broad {
            let potentials = broad.compute(&mut tree.values());
            tree.reindex_values();

            match self.narrow {
                Some(ref mut narrow) => {
                    for (left_entity, right_entity) in potentials {
                        let left_shape = shapes.get(left_entity).unwrap();
                        let right_shape = shapes.get(right_entity).unwrap();
                        let left_pose = poses.get(left_entity).unwrap();
                        let right_pose = poses.get(right_entity).unwrap();
                        match narrow.collide((left_entity, left_shape, left_pose), (
                            right_entity,
                            right_shape,
                            right_pose,
                        )) {
                            Some(contact_set) => contacts.push(contact_set),
                            None => (),
                        };
                    }
                }
                None => {
                    // if we only have a broad phase, we generate contacts for aabb
                    // intersections
                    // right now, we only report the collision, no normal/depth calculation
                    for (left_entity, right_entity) in potentials {
                        contacts.push(ContactSet::new_single(
                            CollisionStrategy::CollisionOnly,
                            (left_entity, right_entity),
                        ));
                    }
                }
            }
        }
    }
}
