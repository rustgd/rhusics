use specs::{Component, VecStorage, Entity};
use cgmath::prelude::*;
use cgmath::BaseFloat;
use collision::{Aabb, MinMax, Discrete};

use std::ops::{DerefMut, Deref};

use collide::{CollisionShape, ContactSet};

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
