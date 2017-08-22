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
    use collide2d::BroadCollisionInfo2D;
    use cgmath::Point2;
    use collision::Aabb2;
    use super::*;

    #[test]
    fn no_intersection_for_miss() {
        let left =
            coll(1, 8., 8., 10., 11.);

        let right =
            coll(2, 12., 13., 18., 18.);

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let left =
            coll(1, 8., 8., 10., 11.);

        let right =
            coll(2, 12., 13., 18., 18.);

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let left =
            coll(1, 8., 8., 10., 11.);

        let right =
            coll(2, 9., 10., 18., 18.);

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![left, right]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 2), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let left =
            coll(1, 8., 8., 10., 11.);

        let right =
            coll(222, 9., 10., 18., 18.);

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![right, left]);
        assert_eq!(1, potentials.len());
        assert_eq!((222, 1), potentials[0]);
    }

    // util
    fn coll(id : u32, min_x : f32, min_y : f32, max_x : f32, max_y : f32) -> BroadCollisionInfo2D<u32, f32> {
        BroadCollisionInfo2D::new(id, bound(min_x, min_y, max_x, max_y))
    }

    fn bound(min_x : f32, min_y : f32, max_x : f32, max_y : f32) -> Aabb2<f32> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
