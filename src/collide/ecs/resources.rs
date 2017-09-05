use std::fmt::Debug;
use std::ops::{DerefMut, Deref};

use cgmath::prelude::*;
use collision::Aabb;
use specs::{Component, VecStorage, Entity};

use Real;
use collide::{CollisionShape, ContactSet};

impl<P, A, T> Component for CollisionShape<P, A, T>
where
    A: Aabb<Scalar = Real> + Send + Sync + 'static,
    A::Diff: Debug + Send + Sync + 'static,
    T: Send + Sync + 'static,
    P: Send + Sync + 'static,
{
    type Storage = VecStorage<CollisionShape<P, A, T>>;
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
