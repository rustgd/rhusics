pub mod gjk;

use collide::{Contact, CollisionShape};

use std::fmt::Debug;
use cgmath::{BaseFloat, VectorSpace, ElementWise, Array, EuclideanSpace, Transform, Decomposed, Rotation};
use collision::{Aabb, Discrete, MinMax};

pub trait NarrowPhase<ID, S, V, P, T, A, R>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    T: Transform<P>,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
{
    fn collide(&mut self,
               left: &mut (CollisionShape<ID, S, V, P, T, A>, Decomposed<V, R>),
               right: &mut (CollisionShape<ID, S, V, P, T, A>, Decomposed<V, R>), ) -> Option<Contact<ID, S, V>>;
}
