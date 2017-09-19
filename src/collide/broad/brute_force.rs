use std::clone::Clone;

use collide::broad::*;

/// Broad phase collision detection brute force implementation.
///
/// Will simply do bounding box intersection tests for all shape combinations.
#[derive(Debug, Default)]
pub struct BruteForce;

impl<D> BroadPhase<D> for BruteForce
where
    D: BroadCollisionData,
    D::Bound: Aabb + Discrete<D::Bound>,
    D::Id: Clone,
{
    fn compute(&mut self, shapes: &mut Vec<D>) -> Vec<(D::Id, D::Id)> {
        let mut pairs = Vec::<(D::Id, D::Id)>::default();
        if shapes.len() <= 1 {
            return pairs;
        }

        for left_index in 0..(shapes.len() - 1) {
            for right_index in 1..shapes.len() {
                if shapes[left_index]
                    .bound()
                    .intersects(&shapes[right_index].bound())
                {
                    pairs.push((
                        shapes[left_index].id().clone(),
                        shapes[right_index].id().clone(),
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
    use Real;
    use collide::broad::BroadCollisionData;

    #[derive(Debug, Clone)]
    pub struct BroadCollisionInfo2 {
        /// The id
        pub id: u32,

        /// The bounding volume
        pub bound: Aabb2<Real>,
        index: usize,
    }

    impl BroadCollisionInfo2 {
        /// Create a new collision info
        pub fn new(id: u32, bound: Aabb2<Real>) -> Self {
            Self {
                id,
                bound,
                index: 0,
            }
        }
    }

    impl BroadCollisionData for BroadCollisionInfo2 {
        type Id = u32;
        type Bound = Aabb2<Real>;

        fn id(&self) -> &u32 {
            &self.id
        }

        fn bound(&self) -> &Aabb2<Real> {
            &self.bound
        }
    }

    #[test]
    fn no_intersection_for_miss() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 12., 13., 18., 18.);

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 12., 13., 18., 18.);

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 9., 10., 18., 18.);

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![left, right]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 2), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(222, 9., 10., 18., 18.);

        let mut brute = BruteForce::default();
        let potentials = brute.compute(&mut vec![right, left]);
        assert_eq!(1, potentials.len());
        assert_eq!((222, 1), potentials[0]);
    }

    // util
    fn coll(id: u32, min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> BroadCollisionInfo2 {
        BroadCollisionInfo2::new(id, bound(min_x, min_y, max_x, max_y))
    }

    fn bound(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Aabb2<Real> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
