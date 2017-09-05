use std::ops::{DerefMut, Deref};

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
/// - `V`: cgmath vector type
#[derive(Debug)]
pub struct Contacts<V>
where
    V: VectorSpace,
{
    contacts: Vec<ContactSet<Entity, V>>,
}

impl<V> Default for Contacts<V>
where
    V: VectorSpace,
{
    fn default() -> Self {
        Self { contacts: Vec::default() }
    }
}

impl<V> Deref for Contacts<V>
where
    V: VectorSpace,
{
    type Target = Vec<ContactSet<Entity, V>>;

    fn deref(&self) -> &Self::Target {
        &self.contacts
    }
}

impl<V> DerefMut for Contacts<V>
where
    V: VectorSpace,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.contacts
    }
}
