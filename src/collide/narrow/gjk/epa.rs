use collide::{CollisionPrimitive, Contact, CollisionStrategy};
use cgmath::prelude::*;
use cgmath::{Decomposed, BaseFloat};
use collision::{Aabb, MinMax, Discrete};

#[allow(dead_code)]
const EPA_TOLERANCE: f32 = 0.00001;

#[allow(unused_variables)]
pub fn epa<S, V, P, R, A>(
    simplex: &mut Vec<V>,
    left: &CollisionPrimitive<S, V, P, R, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, R, A>,
    right_transform: &Decomposed<V, R>,
) -> Vec<Contact<S, V>>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
{
    vec![Contact::new(CollisionStrategy::FullResolution)] // TODO
}
