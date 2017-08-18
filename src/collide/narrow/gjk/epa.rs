use collide::{CollisionPrimitive, Contact, CollisionStrategy};
use cgmath::{Decomposed, BaseFloat, VectorSpace, ElementWise, Array, InnerSpace, EuclideanSpace, Rotation};
use collision::{Aabb, MinMax, Discrete};

use std::fmt::Debug;

const EPA_TOLERANCE: f32 = 0.00001;

pub fn epa<ID, S, V, P, R, A>(
    bodies: (ID, ID),
    simples: &mut Vec<V>,
    left: &CollisionPrimitive<S, V, P, Decomposed<V, R>, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, Decomposed<V, R>, A>,
    right_transform: &Decomposed<V, R>,
) -> Contact<ID, S, V>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
{
    Contact::new(CollisionStrategy::FullResolution, bodies) // TODO
}
