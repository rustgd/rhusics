pub mod gjk;

use collide::{ContactSet, CollisionShape};

use std::fmt::Debug;
use cgmath::Decomposed;
use collision::Aabb;

pub trait NarrowPhase<ID, P, A, R>
where
    A: Aabb,
    A::Diff: Debug,
{
    fn collide(
        &mut self,
        left: (ID, &CollisionShape<P, A, R>, &Decomposed<A::Diff, R>),
        right: (ID, &CollisionShape<P, A, R>, &Decomposed<A::Diff, R>),
    ) -> Option<ContactSet<ID, A::Diff>>;
}
