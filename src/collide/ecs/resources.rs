use std::ops::{DerefMut, Deref};
use std::fmt::Debug;

use cgmath::prelude::*;
use specs::{Component, VecStorage, Entity};

use collide::{CollisionShape, ContactSet, Primitive};

impl<P, T> Component for CollisionShape<P, T>
where
    T: Send + Sync + 'static,
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Send + Sync + 'static,
{
    type Storage = VecStorage<CollisionShape<P, T>>;
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
    contacts: Vec<ContactSet<Entity, P>>,
}

impl<P> Default for Contacts<P>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    fn default() -> Self {
        Self { contacts: Vec::default() }
    }
}

impl<P> Deref for Contacts<P>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    type Target = Vec<ContactSet<Entity, P>>;

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
