pub mod gjk;

use collide::{Contact, CollisionShape};

use std::fmt::Debug;
use cgmath::{BaseFloat, VectorSpace, ElementWise, Array, EuclideanSpace, Decomposed, Rotation};
use collision::{Aabb, Discrete, MinMax};

pub trait NarrowPhase<ID, S, V, P, R, A>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
{
    fn collide(&mut self,
               left: (ID, &CollisionShape<S, V, P, R, A>, &Decomposed<V, R>),
               right: (ID, &CollisionShape<S, V, P, R, A>, &Decomposed<V, R>), )
        -> Vec<Contact<ID, S, V>>;
}
