use collide::{CollisionPrimitive, Contact, CollisionStrategy, Primitive};
use super::simplex::SimplexProcessor;
use super::support;
use cgmath::prelude::*;
use cgmath::{Decomposed, BaseFloat};
use cgmath::num_traits::NumCast;
use collision::Aabb;

use std::ops::Neg;

const EPA_TOLERANCE: f32 = 0.00001;
const MAX_ITERATIONS: u32 = 100;

pub fn epa<P, A, R, S>(
    simplex: &mut Vec<A::Diff>,
    left: &CollisionPrimitive<P, A, R>,
    left_transform: &Decomposed<A::Diff, R>,
    right: &CollisionPrimitive<P, A, R>,
    right_transform: &Decomposed<A::Diff, R>,
    processor: &S,
) -> Vec<Contact<A::Diff>>
where
    A: Aabb + Clone,
    P: Primitive<A>,
    A::Scalar: BaseFloat,
    A::Diff: InnerSpace + Neg<Output = A::Diff>,
    R: Rotation<A::Point>,
    S: SimplexProcessor<A::Scalar, Vector = A::Diff>,
{
    let mut i = 0;
    let tolerance: A::Scalar = NumCast::from(EPA_TOLERANCE).unwrap();
    if processor.closest_feature(&simplex).is_none() {
        return Vec::default();
    }

    loop {
        let e = processor.closest_feature(&simplex);
        let e = e.unwrap();
        let p = support(left, left_transform, right, right_transform, &e.normal);
        let d = p.dot(e.normal);
        if d - e.distance < tolerance {
            return vec![
                Contact::new_impl(CollisionStrategy::FullResolution, e.normal, e.distance),
            ];
        } else {
            simplex.insert(e.index, p);
        }
        i += 1;
        if i >= MAX_ITERATIONS {
            return vec![
                Contact::new_impl(CollisionStrategy::FullResolution, e.normal, e.distance),
            ];
        }
    }
}

/*
#[cfg(test)]
mod tests {

    use super::*;
    use cgmath::{Vector2, Decomposed, Rotation2, Rad};
    use collide2d::*;

    #[test]
    fn test_simplex_0() {}
}
*/