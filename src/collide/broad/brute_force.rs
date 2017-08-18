use collide::broad::*;

use std::fmt::Debug;
use std::clone::Clone;

#[derive(Debug, Default)]
pub struct BruteForce;

impl<ID, S, V, P, A> BroadPhase<ID, S, V, P, A> for BruteForce
where
    ID: Debug + Clone,
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V>
        + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
{
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID, S, V, P, A>>) -> Vec<(ID, ID)> {
        let mut pairs = Vec::<(ID, ID)>::default();
        if shapes.len() <= 1 {
            return pairs;
        }

        for left_index in 0..(shapes.len() - 1) {
            for right_index in 1..shapes.len() {
                if shapes[left_index].bound.intersects(
                    &shapes[right_index].bound,
                )
                {
                    pairs.push((
                        shapes[left_index].id.clone(),
                        shapes[right_index].id.clone(),
                    ));
                }
            }
        }
        pairs
    }
}

#[cfg(test)]
mod tests {
    use cgmath::Point2;
    use collision::Aabb2;
    use super::*;

    #[test]
    fn no_intersection_for_miss() {
        let left =
            BroadCollisionInfo::new(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right =
            BroadCollisionInfo::new(2, Aabb2::new(Point2::new(12., 13.), Point2::new(18., 18.)));

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let left =
            BroadCollisionInfo::new(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right =
            BroadCollisionInfo::new(2, Aabb2::new(Point2::new(12., 13.), Point2::new(18., 18.)));

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let left =
            BroadCollisionInfo::new(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right =
            BroadCollisionInfo::new(2, Aabb2::new(Point2::new(9., 10.), Point2::new(18., 18.)));

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![left, right]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 2), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let left =
            BroadCollisionInfo::new(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right =
            BroadCollisionInfo::new(222, Aabb2::new(Point2::new(9., 10.), Point2::new(18., 18.)));

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![right, left]);
        assert_eq!(1, potentials.len());
        assert_eq!((222, 1), potentials[0]);
    }
}
