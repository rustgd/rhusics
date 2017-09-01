use std::fmt::Debug;

use collision::Aabb;
use specs::{System, ReadStorage, Join, WriteStorage, Entities, Entity, FetchMut, Component};

use {Pose, Real};
use collide::{CollisionShape, CollisionStrategy, ContactSet, Primitive};
use collide::broad::{BroadPhase, BroadCollisionInfo};
use collide::ecs::resources::Contacts;
use collide::narrow::NarrowPhase;

pub struct CollisionSystem<P, A, T>
where
    A: Aabb + Clone,
    A::Diff: Debug,
{
    narrow: Option<Box<NarrowPhase<Entity, P, A, T>>>,
    broad: Option<Box<BroadPhase<Entity, A>>>,
}

impl<P, A, T> CollisionSystem<P, A, T>
where
    A: Aabb<Scalar = Real> + Clone + Send + Sync + 'static,
    A::Diff: Debug + Send + Sync + 'static,
    A::Point: Send + Sync + 'static,
    P: Primitive<A> + Send + Sync + 'static,
    T: Pose<A::Point> + Component,
{
    pub fn new() -> Self {
        Self {
            narrow: None,
            broad: None,
        }
    }

    pub fn with_narrow_phase<N: NarrowPhase<Entity, P, A, T> + 'static>(
        mut self,
        narrow: N,
    ) -> Self {
        self.narrow = Some(Box::new(narrow));
        self
    }

    pub fn with_broad_phase<B: BroadPhase<Entity, A> + 'static>(mut self, broad: B) -> Self {
        self.broad = Some(Box::new(broad));
        self
    }
}

impl<'a, P, A, T> System<'a> for CollisionSystem<P, A, T>
where
    A: Aabb<Scalar = Real>
        + Clone
        + Send
        + Sync
        + 'static,
    A::Diff: Debug + Send + Sync + 'static,
    A::Point: Send + Sync + 'static,
    P: Primitive<A> + Send + Sync + 'static,
    T: Component
        + Pose<A::Point>
        + Send
        + Sync
        + 'static,
{
    type SystemData = (Entities<'a>,
     ReadStorage<'a, T>,
     WriteStorage<'a, CollisionShape<P, A, T>>,
     FetchMut<'a, Contacts<A::Diff>>);

    fn run(&mut self, (entities, poses, mut shapes, mut contacts): Self::SystemData) {
        contacts.clear();
        match self.broad {
            Some(ref mut broad) => {
                let mut info: Vec<BroadCollisionInfo<Entity, A>> = Vec::default();
                for (entity, pose, shape) in (&*entities, &poses, &mut shapes).join() {
                    shape.update(&pose);
                    info.push(BroadCollisionInfo::new(
                        entity,
                        shape.transformed_bound.clone(),
                    ));
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
            None => (), // we only do narrow phase if we have both a broad phase and a narrow phase
        };
    }
}
