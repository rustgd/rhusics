use std::fmt::Debug;

use cgmath::prelude::*;
use collision::prelude::*;
use shrev::EventChannel;
use specs::{Component, Entities, Entity, FetchMut, Join, ReadStorage, System, WriteStorage};

use {NextFrame, Real};
use collide::{CollisionShape, CollisionStrategy, ContactEvent, Primitive};
use collide::broad::{BroadPhase, HasBound};
use collide::narrow::NarrowPhase;
use ecs::collide::resources::GetEntity;

/// Collision detection [system](https://docs.rs/specs/0.9.5/specs/trait.System.html) for use with
/// [`specs`](https://docs.rs/specs/0.9.5/specs/).
///
/// Has support for both broad phase and narrow phase collision detection. Will only do narrow phase
/// if both broad and narrow phase is activated.
///
/// Can handle any transform component type, as long as the type implements
/// [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html), and as long as the
/// storage is wrapped in a
/// [`FlaggedStorage`](https://docs.rs/specs/0.9.5/specs/struct.FlaggedStorage.html).
///
/// ### Type parameters:
///
/// - `P`: Shape primitive
/// - `T`: Transform
/// - `D`: Data accepted by broad phase
/// - `Y`: Shape type, see `Collider`
pub struct BasicCollisionSystem<P, T, D, Y = ()>
where
    P: Primitive,
    P::Aabb: Clone + Debug + Aabb<Scalar = Real>,
{
    narrow: Option<Box<NarrowPhase<P, T, Y>>>,
    broad: Option<Box<BroadPhase<D>>>,
}

impl<P, T, D, Y> BasicCollisionSystem<P, T, D, Y>
where
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Aabb<Scalar = Real> + Clone + Debug + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Diff: Debug,
    T: Transform<P::Point> + Component,
    D: HasBound<Bound = P::Aabb>,
{
    /// Create a new collision detection system, with no broad or narrow phase activated.
    pub fn new() -> Self {
        Self {
            narrow: None,
            broad: None,
        }
    }

    /// Specify what narrow phase algorithm to use
    pub fn with_narrow_phase<N: NarrowPhase<P, T, Y> + 'static>(mut self, narrow: N) -> Self {
        self.narrow = Some(Box::new(narrow));
        self
    }

    /// Specify what broad phase algorithm to use
    pub fn with_broad_phase<B: BroadPhase<D> + 'static>(mut self, broad: B) -> Self {
        self.broad = Some(Box::new(broad));
        self
    }
}

impl<'a, P, T, Y, D> System<'a> for BasicCollisionSystem<P, T, D, Y>
where
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Aabb<Scalar = Real> + Clone + Debug + Send + Sync + 'static,
    P::Point: Debug + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
    T: Component + Transform<P::Point> + Send + Sync + Clone + 'static,
    Y: Default + Send + Sync + 'static,
    for<'b: 'a> D: HasBound<Bound = P::Aabb>
        + From<(Entity, &'b CollisionShape<P, T, Y>)>
        + GetEntity,
{
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, T>,
        ReadStorage<'a, NextFrame<T>>,
        WriteStorage<'a, CollisionShape<P, T, Y>>,
        FetchMut<'a, EventChannel<ContactEvent<Entity, P::Point>>>,
    );

    fn run(&mut self, system_data: Self::SystemData) {
        let (entities, poses, next_poses, mut shapes, mut event_channel) = system_data;

        if let Some(ref mut broad) = self.broad {
            let mut info = Vec::default();
            for (entity, pose, shape) in (&*entities, &poses, &mut shapes).join() {
                shape.update(&pose, next_poses.get(entity).map(|p| &p.value));
                info.push((entity, &*shape).into());
            }
            let potentials = broad
                .find_potentials(&mut info)
                .iter()
                .map(|&(a, b)| (info[a].entity(), info[b].entity()))
                .collect::<Vec<_>>();

            match self.narrow {
                Some(ref mut narrow) => for (left_entity, right_entity) in potentials {
                    let left_shape = shapes.get(left_entity).unwrap();
                    let right_shape = shapes.get(right_entity).unwrap();
                    let left_pose = poses.get(left_entity).unwrap();
                    let right_pose = poses.get(right_entity).unwrap();
                    match narrow.collide(left_shape, left_pose, right_shape, right_pose) {
                        Some(contact) => {
                            event_channel.single_write(ContactEvent::new(
                                (left_entity.clone(), right_entity.clone()),
                                contact,
                            ));
                        }
                        None => (),
                    };
                },
                None => {
                    // if we only have a broad phase, we generate contacts for aabb
                    // intersections
                    // right now, we only report the collision, no normal/depth calculation
                    for (left_entity, right_entity) in potentials {
                        event_channel.single_write(ContactEvent::new_single(
                            CollisionStrategy::CollisionOnly,
                            (left_entity, right_entity),
                        ));
                    }
                }
            }
        }
    }
}
