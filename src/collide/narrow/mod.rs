pub mod gjk;

use std::fmt::Debug;

use collision::Aabb;

use collide::{ContactSet, CollisionShape};

pub trait NarrowPhase<ID, P, A, T>
where
    A: Aabb,
    A::Diff: Debug,
{
    fn collide(
        &mut self,
        left: (ID, &CollisionShape<P, A, T>, &T),
        right: (ID, &CollisionShape<P, A, T>, &T),
    ) -> Option<ContactSet<ID, A::Diff>>;
}
