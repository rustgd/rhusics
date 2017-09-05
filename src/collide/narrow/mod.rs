pub use self::gjk::{GJK, EPA2D, EPA3D, SimplexProcessor2D, SimplexProcessor3D};

use std::fmt::Debug;

use collision::Aabb;

use collide::{ContactSet, CollisionShape};

mod gjk;

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
