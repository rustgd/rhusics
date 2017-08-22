use specs::{System, ReadStorage, Join, Component, VecStorage, WriteStorage, Entities, Entity,
            FetchMut};
use cgmath::prelude::*;
use cgmath::{BaseFloat, Decomposed};
use collision::{Aabb, MinMax, Discrete};

use std::fmt::Debug;
use std::ops::{DerefMut, Deref};

use collide::{CollisionShape, CollisionStrategy, ContactSet};
use collide::narrow::NarrowPhase;
use collide::broad::{BroadPhase, BroadCollisionInfo};

use BodyPose;

impl<S, V, P, R, A> Component for CollisionShape<S, V, P, R, A>
where
    S: BaseFloat
        + Send
        + Sync
        + 'static,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>
        + Send
        + Sync
        + 'static,
    P: EuclideanSpace<
        Scalar = S,
        Diff = V,
    >
        + MinMax
        + Send
        + Sync
        + 'static,
    R: Rotation<P>
        + Send
        + Sync
        + 'static,
    A: Aabb<S, V, P>
        + Discrete<A>
        + Send
        + Sync
        + 'static,
{
    type Storage = VecStorage<CollisionShape<S, V, P, R, A>>;
}

#[derive(Debug)]
pub struct Contacts<S, V>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
{
    contacts: Vec<ContactSet<Entity, S, V>>,
}

impl<S, V> Contacts<S, V>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
{
    pub fn new() -> Self {
        Self { contacts: Vec::default() }
    }
}

impl<S, V> Deref for Contacts<S, V>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
{
    type Target = Vec<ContactSet<Entity, S, V>>;

    fn deref(&self) -> &Self::Target {
        &self.contacts
    }
}

impl<S, V> DerefMut for Contacts<S, V>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.contacts
    }
}

pub struct CollisionSystem<S, V, P, R, A>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    R: Rotation<P>,
    A: Aabb<S, V, P> + Discrete<A>,
{
    narrow: Option<Box<NarrowPhase<Entity, S, V, P, R, A>>>,
    broad: Option<Box<BroadPhase<Entity, S, V, P, A>>>,
}

impl<S, V, P, R, A> CollisionSystem<S, V, P, R, A>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    R: Rotation<P>,
    A: Aabb<S, V, P> + Discrete<A>,
{
    pub fn new() -> Self {
        Self {
            narrow: None,
            broad: None,
        }
    }

    pub fn with_narrow_phase<N: NarrowPhase<Entity, S, V, P, R, A> + 'static>(
        mut self,
        narrow: N,
    ) -> Self {
        self.narrow = Some(Box::new(narrow));
        self
    }

    pub fn with_broad_phase<B: BroadPhase<Entity, S, V, P, A> + 'static>(
        mut self,
        broad: B,
    ) -> Self {
        self.broad = Some(Box::new(broad));
        self
    }
}

impl<'a, S, V, P, R, A> System<'a> for CollisionSystem<S, V, P, R, A>
where
    S: BaseFloat
        + Send
        + Sync
        + 'static
        + Debug,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>
        + Send
        + Sync
        + 'static
        + Debug,
    P: EuclideanSpace<
        Scalar = S,
        Diff = V,
    >
        + MinMax
        + Send
        + Sync
        + 'static
        + Debug,
    R: Rotation<P>
        + Send
        + Sync
        + 'static
        + Debug,
    A: Aabb<S, V, P>
        + Discrete<A>
        + Send
        + Sync
        + 'static
        + Clone
        + Debug,
{
    type SystemData = (Entities<'a>,
     ReadStorage<'a, BodyPose<S, V, P, R>>,
     WriteStorage<'a, CollisionShape<S, V, P, R, A>>,
     FetchMut<'a, Contacts<S, V>>);

    fn run(&mut self, (entities, poses, mut shapes, mut contacts): Self::SystemData) {
        contacts.clear();
        match self.broad {
            Some(ref mut broad) => {
                let mut info: Vec<BroadCollisionInfo<Entity, S, V, P, A>> = Vec::default();
                for (entity, pose, shape) in (&*entities, &poses, &mut shapes).join() {
                    if pose.dirty {
                        let c: Decomposed<V, R> = pose.into();
                        shape.update(&c);
                    }
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
                            match narrow.collide((left_entity, left_shape, &left_pose.into()), (
                                right_entity,
                                right_shape,
                                &right_pose
                                    .into(),
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
