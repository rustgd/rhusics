use collide::{CollisionPrimitive, Contact, CollisionStrategy};
use super::simplex::SimplexProcessor;
use super::support;
use cgmath::prelude::*;
use cgmath::{Decomposed, BaseFloat};
use collision::{Aabb, MinMax, Discrete};

use std::fmt::Debug;
use std::ops::Neg;

#[allow(dead_code)]
const EPA_TOLERANCE: f32 = 0.00001;
const MAX_ITERATIONS: u32 = 100;

#[allow(unused_variables)]
pub fn epa<S, V, P, R, A, SP>(
    simplex: &mut Vec<V>,
    left: &CollisionPrimitive<S, V, P, R, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, R, A>,
    right_transform: &Decomposed<V, R>,
    processor: &SP,
) -> Vec<Contact<S, V>>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace + Neg<Output=V>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax + Debug,
    A: Aabb<S, V, P> + Discrete<A> + Clone,
    R: Rotation<P>,
    SP: SimplexProcessor<S, V>,
{
    let mut i = 0;
    let tolerance = S::from(EPA_TOLERANCE).unwrap();
    if processor.closest_feature(&simplex).is_none() {
        return Vec::default();
    }
    loop {
        let e = processor.closest_feature(&simplex);
        let e = e.unwrap();
        let p = support(left, left_transform, right, right_transform, &e.normal);
        let d = p.dot(e.normal);
        if d - e.distance < tolerance {
            return vec![Contact::new_impl(
                CollisionStrategy::FullResolution, e.normal, e.distance)]
        } else {
            simplex.insert(e.index, p);
        }
        i += 1;
        if i >= MAX_ITERATIONS {
            return vec![Contact::new_impl(
                CollisionStrategy::FullResolution, e.normal, e.distance)]
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use cgmath::{Vector2, Decomposed, Rotation2, Rad};
    use collide2d::*;

    #[test]
    fn test_simplex_0() {

    }
}