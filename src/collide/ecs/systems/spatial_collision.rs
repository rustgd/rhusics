use std::fmt::Debug;

use collision::dbvt::{DiscreteVisitor, DynamicBoundingVolumeTree};
use collision::prelude::*;
use shrev::EventHandler;
use specs::{Component, Entities, Entity, FetchMut, Join, ReadStorage, System};

use {Pose, Real};
use collide::{CollisionShape, CollisionStrategy, ContactSet, ContainerShapeWrapper, Primitive};
use collide::broad::{BroadCollisionData, BroadPhase};
use collide::ecs::resources::Contacts;
use collide::narrow::NarrowPhase;

/// Collision detection [system](https://docs.rs/specs/0.9.5/specs/trait.System.html) for use with
/// [`specs`](https://docs.rs/specs/0.9.5/specs/).
///
/// Will perform spatial sorting of the collision world.
///
/// Has support for both broad phase and narrow phase collision detection. Will only do narrow phase
/// if both broad and narrow phase is activated. If no broad phase is set, it will use a DBVT based
/// broad phase that has complexity O(m log^2 n), where m is the number of shapes that have a dirty
/// pose.
///
/// Can handle any transform component type, as long as the type implements
/// [`Pose`](../../trait.Pose.html) and
/// [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html).
///
#[derive(Debug)]
pub struct SpatialCollisionSystem<P, T, D>
where
    P: Primitive,
    P::Aabb: Clone + Debug,
{
    narrow: Option<Box<NarrowPhase<Entity, P, T>>>,
    broad: Option<Box<BroadPhase<D>>>,
}

impl<P, T, D> SpatialCollisionSystem<P, T, D>
where
    P: Primitive + Send + Sync + 'static,
    P::Vector: Debug,
    P::Aabb: Clone
        + Debug
        + Send
        + Sync
        + 'static
        + Union<P::Aabb, Output = P::Aabb>
        + Contains<P::Aabb>
        + SurfaceArea<Scalar = Real>,
    T: Pose<P::Point> + Component,
    D: BroadCollisionData<Bound = P::Aabb, Id = Entity>,
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
    pub fn with_broad_phase<B: BroadPhase<D> + 'static>(mut self, broad: B) -> Self {
        self.broad = Some(Box::new(broad));
        self
    }
}

fn discrete_visitor<P>(
    bound: &P::Aabb,
) -> DiscreteVisitor<P::Aabb, ContainerShapeWrapper<Entity, P>>
where
    P: Primitive,
    P::Aabb: Debug + Discrete<P::Aabb>,
    P::Vector: Debug,
{
    DiscreteVisitor::<P::Aabb, ContainerShapeWrapper<Entity, P>>::new(bound)
}

impl<'a, P, T> System<'a>
    for SpatialCollisionSystem<P, T, (usize, ContainerShapeWrapper<Entity, P>)>
where
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Clone
        + Debug
        + Send
        + Sync
        + 'static
        + Discrete<P::Aabb>
        + Contains<P::Aabb>
        + SurfaceArea<Scalar = Real>,
    P::Vector: Debug + Send + Sync + 'static,
    P::Point: Debug + Send + Sync + 'static,
    T: Component + Clone + Debug + Pose<P::Point> + Send + Sync + 'static,
{
    type SystemData = (Entities<'a>,
     ReadStorage<'a, T>,
     ReadStorage<'a, CollisionShape<P, T>>,
     Option<FetchMut<'a, Contacts<P::Point>>>,
     Option<FetchMut<'a, EventHandler>>,
     FetchMut<'a, DynamicBoundingVolumeTree<ContainerShapeWrapper<Entity, P>>>);

    fn run(
        &mut self,
        (entities, poses, shapes, mut contacts, mut event_handler, mut tree): Self::SystemData,
    ) {
        if let Some(ref mut c) = contacts {
            c.clear();
        }

        let potentials = if let Some(ref mut broad) = self.broad {
            // Overridden broad phase, use that
            let potentials = broad.compute(tree.values());
            tree.reindex_values();
            potentials
        } else {
            // Fallback to DBVT based broad phase
            let mut potentials = Vec::default();
            // find changed values, do intersection tests against tree for each
            for (entity, pose, shape) in (&*entities, &poses, &shapes).join() {
                if pose.dirty() {
                    for (v, _) in tree.query(&mut discrete_visitor::<P>(shape.bound())) {
                        if entity != v.id {
                            let n = if entity < v.id {
                                (entity, v.id.clone())
                            } else {
                                (v.id.clone(), entity)
                            };
                            let pos = match potentials.binary_search(&n) {
                                Err(pos) => Some(pos),
                                Ok(_) => None,
                            };
                            if let Some(pos) = pos {
                                potentials.insert(pos, n);
                            }
                        }
                    }
                }
            }
            potentials
        };

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
                        Some(contact_set) => {
                            if let Some(ref mut events) = event_handler {
                                match events.write_single(contact_set) {
                                    Err(err) => println!("Error in event write: {:?}", err),
                                    _ => (),
                                };
                            } else if let Some(ref mut c) = contacts {
                                c.push(contact_set);
                            }
                        }
                        None => (),
                    };
                }
            }
            None => {
                // if we only have a broad phase, we generate contacts for aabb
                // intersections
                // right now, we only report the collision, no normal/depth calculation
                for (left_entity, right_entity) in potentials {
                    let contact_set = ContactSet::new_single(
                        CollisionStrategy::CollisionOnly,
                        (left_entity, right_entity),
                    );
                    if let Some(ref mut events) = event_handler {
                        match events.write_single(contact_set) {
                            Err(err) => println!("Error in event write: {:?}", err),
                            _ => (),
                        };
                    } else if let Some(ref mut c) = contacts {
                        c.push(contact_set);
                    }
                }
            }
        }
    }
}
