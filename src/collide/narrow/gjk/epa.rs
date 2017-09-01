use std::ops::Neg;

use cgmath::prelude::*;
use collision::Aabb;

use super::simplex::SimplexProcessor;
use super::support;
use {Pose, Real};
use collide::{CollisionPrimitive, Contact, CollisionStrategy, Primitive};

const EPA_TOLERANCE: f32 = 0.00001;
const MAX_ITERATIONS: u32 = 100;

pub fn epa<P, A, T, S>(
    simplex: &mut Vec<A::Diff>,
    left: &CollisionPrimitive<P, A, T>,
    left_transform: &T,
    right: &CollisionPrimitive<P, A, T>,
    right_transform: &T,
    processor: &S,
) -> Vec<Contact<A::Diff>>
where
    A: Aabb<Scalar = Real> + Clone,
    P: Primitive<A>,
    A::Diff: InnerSpace + Neg<Output = A::Diff>,
    T: Pose<A::Point>,
    S: SimplexProcessor<Vector = A::Diff>,
{
    let mut i = 0;
    let tolerance = EPA_TOLERANCE as Real;
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
