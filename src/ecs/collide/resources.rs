use std::fmt::Debug;
use std::ops::{Deref, DerefMut};

use cgmath::prelude::*;
use collision::{Aabb, Primitive};
use specs::{Component, DenseVecStorage, Entity, FlaggedStorage};

use {BodyPose, NextFrame, Real};
use collide::{CollisionShape, ContactEvent};
use collide::util::ContainerShapeWrapper;

impl<P, R> Component for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
{
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<T> Component for NextFrame<T>
where
    T: Send + Sync + 'static,
{
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

/// Retrieve the entity for the given object
pub trait GetEntity {
    /// Return the entity
    fn entity(&self) -> Entity;
}

impl<P, T> Component for CollisionShape<P, T>
where
    T: Send + Sync + 'static,
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Send + Sync + 'static,
{
    type Storage = DenseVecStorage<CollisionShape<P, T>>;
}

/// Contacts storage for use in ECS.
///
/// Will typically contain the contacts found in the last collision detection run.
///
/// # Type parameters:
///
/// - `P`: cgmath point type
#[derive(Debug)]
pub struct Contacts<P>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    contacts: Vec<ContactEvent<Entity, P>>,
}

impl<P> Default for Contacts<P>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    fn default() -> Self {
        Self {
            contacts: Vec::default(),
        }
    }
}

impl<P> Deref for Contacts<P>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    type Target = Vec<ContactEvent<Entity, P>>;

    fn deref(&self) -> &Self::Target {
        &self.contacts
    }
}

impl<P> DerefMut for Contacts<P>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.contacts
    }
}

impl<'a, P, T> From<(Entity, &'a CollisionShape<P, T>)> for ContainerShapeWrapper<Entity, P>
where
    P: Primitive,
    P::Aabb: Aabb<Scalar = Real>,
    <P::Point as EuclideanSpace>::Diff: Debug,
    T: Transform<P::Point>,
{
    fn from((entity, ref shape): (Entity, &CollisionShape<P, T>)) -> Self {
        Self::new(entity, shape.bound())
    }
}

impl<P> GetEntity for ContainerShapeWrapper<Entity, P>
where
    P: Primitive,
    P::Aabb: Aabb<Scalar = Real>,
    <P::Point as EuclideanSpace>::Diff: Debug,
{
    fn entity(&self) -> Entity {
        self.id
    }
}
