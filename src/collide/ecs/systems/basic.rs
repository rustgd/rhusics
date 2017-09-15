use std::fmt::Debug;

use specs::{System, ReadStorage, Join, WriteStorage, Entities, Entity, FetchMut, Component};

use Pose;
use collide::{CollisionShape, CollisionStrategy, ContactSet, Primitive, ContainerShapeWrapper};
use collide::broad::{BroadPhase, BroadCollisionData};
use collide::ecs::resources::Contacts;
use collide::narrow::NarrowPhase;

/// Collision detection [system](https://docs.rs/specs/0.9.5/specs/trait.System.html) for use with
/// [`specs`](https://docs.rs/specs/0.9.5/specs/).
///
/// Has support for both broad phase and narrow phase collision detection. Will only do narrow phase
/// if both broad and narrow phase is activated.
///
/// Can handle any transform component type, as long as the type implements
/// [`Pose`](../../trait.Pose.html) and
/// [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html).
///
#[derive(Debug)]
pub struct BasicCollisionSystem<P, T, D>
where
    P: Primitive,
    P::Aabb: Clone + Debug,
{
    narrow: Option<Box<NarrowPhase<Entity, P, T>>>,
    broad: Option<Box<BroadPhase<D>>>,
}

impl<P, T, D> BasicCollisionSystem<P, T, D>
where
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Clone + Debug + Send + Sync + 'static,
    P::Vector: Debug,
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

impl<'a, P, T> System<'a> for BasicCollisionSystem<P, T, ContainerShapeWrapper<Entity, P>>
    where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Clone
        + Debug
        + Send
        + Sync
        + 'static,
        P::Point: Debug + Send + Sync + 'static,
        P::Vector: Debug + Send + Sync + 'static,
        T: Component
        + Pose<P::Point>
        + Send
        + Sync
        + Clone
        + 'static,
{
    type SystemData = (Entities<'a>,
                       ReadStorage<'a, T>,
                       WriteStorage<'a, CollisionShape<P, T>>,
                       FetchMut<'a, Contacts<P::Point>>);

    fn run(&mut self, (entities, poses, mut shapes, mut contacts): Self::SystemData) {
        contacts.clear();

        if let Some(ref mut broad) = self.broad {

            let mut info: Vec<ContainerShapeWrapper<Entity, P>> = Vec::default();
            for (entity, pose, shape) in (&*entities, &poses, &mut shapes).join() {
                shape.update(&pose);
                info.push(ContainerShapeWrapper::new(entity, shape.bound()));
            }
            let potentials = broad.compute(&mut info);

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
