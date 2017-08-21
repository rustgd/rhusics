use collision::{Aabb, Discrete, MinMax};
use cgmath::prelude::*;
use cgmath::BaseFloat;
use collide::CollisionShape;

pub mod sweep_prune;
pub mod brute_force;

use std::fmt::Debug;
use std::clone::Clone;
use std;

#[derive(Debug)]
pub struct BroadCollisionInfo<ID, S, V, P, A>
where
    ID: Debug + Clone,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
{
    id: ID,
    bound: A,
    m: std::marker::PhantomData<(S, V, P)>,
}

impl<ID, S, V, P, A> BroadCollisionInfo<ID, S, V, P, A>
where
    ID: Debug + Clone,
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
{
    pub fn new(id: ID, bound: A) -> Self {
        Self {
            id,
            bound,
            m: std::marker::PhantomData,
        }
    }
}

impl<ID, S, V, P, R, A> From<(ID, CollisionShape<S, V, P, R, A>)>
    for BroadCollisionInfo<ID, S, V, P, A>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    R: Rotation<P>,
    A: Aabb<S, V, P> + Discrete<A> + Clone,
{
    fn from((id, shape): (ID, CollisionShape<S, V, P, R, A>)) -> Self {
        Self::new(id.clone(), shape.transformed_bound.clone())
    }
}

pub trait BroadPhase<ID, S, V, P, A>
where
    ID: Debug + Clone,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
{
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID, S, V, P, A>>) -> Vec<(ID, ID)>;
}
