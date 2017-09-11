pub use self::variance::*;

use std::clone::Clone;
use std::cmp::Ordering;
use std::fmt::Debug;

use Real;
use collide::broad::*;

/// Sweep and prune broad phase collision detection algorithm.
///
/// Will sort the bounding boxes of the collision world along some axis, and will then sweep the
/// sorted list, and compare the bounds along the sweep axis, advancing a
/// "can still be collided with" index when the end point is reached. If shapes are approximately
/// the same size throughout the world, this will give close to linear time. If some of the shapes
/// are much larger than the others, this will cause degradation down to quadratic time.
///
/// Any shape pairs found by the base algorithm, will then do a bounding box intersection test,
/// before adding to the resulting pairs list.
///
/// # Type parameters:
///
/// - `V`: Variance type used for computing what axis to use on the next iteration. Should be either
///        [`Variance2`](struct.Variance2.html) or [`Variance3`](struct.Variance3.html)
#[derive(Debug)]
pub struct SweepAndPrune<V> {
    sweep_axis: usize,
    variance: V,
}

impl<V> SweepAndPrune<V>
where
    V: Variance,
{
    /// Create a new sweep and prune algorithm, will use the X axis as the first sweep axis
    pub fn new() -> Self {
        Self::new_impl(0)
    }

    /// Create a new sweep and prune algorithm, starting with the given axis as the first sweep axis
    pub fn new_impl(sweep_axis: usize) -> Self {
        Self {
            sweep_axis,
            variance: V::new(),
        }
    }
}

impl<ID, A, V> BroadPhase<ID, A> for SweepAndPrune<V>
where
    ID: Clone + Debug,
    A: Aabb<Scalar = Real> + Discrete<A> + Debug,
    A::Point: EuclideanSpace,
    A::Diff: VectorSpace + ElementWise,
    V: Variance<Point = A::Point> + Debug,
{
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID, A>>) -> Vec<(ID, ID)> {
        let mut pairs = Vec::<(ID, ID)>::default();
        if shapes.len() <= 1 {
            return pairs;
        }

        shapes.sort_by(|a, b| if a.bound.min()[self.sweep_axis] !=
            b.bound.min()[self.sweep_axis]
        {
            a.bound.min()[self.sweep_axis]
                .partial_cmp(&b.bound.min()[self.sweep_axis])
                .unwrap_or(Ordering::Equal)
        } else {
            a.bound.max()[self.sweep_axis]
                .partial_cmp(&b.bound.max()[self.sweep_axis])
                .unwrap_or(Ordering::Equal)
        });

        self.variance.clear();
        self.variance.add_to_sum(
            &shapes[0].bound.min(),
            &shapes[0].bound.max(),
        );

        let mut active = vec![0];
        for index in 1..shapes.len() {
            let mut i = 0;
            // for all currently active bounds, go through and remove any that are to the left of
            // the current bound, any others are potential hits, do a real bound intersection test
            // for those, and add to pairs if the bounds intersect.
            while i < active.len() {
                if shapes[active[i]].bound.max()[self.sweep_axis] <
                    shapes[index].bound.min()[self.sweep_axis]
                {
                    active.remove(i);
                } else {
                    if shapes[active[i]].bound.intersects(&shapes[index].bound) {
                        pairs.push((shapes[active[i]].id.clone(), shapes[index].id.clone()));
                    }
                    i += 1;
                }
            }

            // current bound should be active for the next iteration
            active.push(index);

            // update variance
            self.variance.add_to_sum(
                &shapes[index].bound.min(),
                &shapes[index].bound.max(),
            );
        }

        // compute sweep axis for the next iteration
        let (axis, _) = self.variance.compute_axis(shapes.len() as Real);
        self.sweep_axis = axis;

        pairs
    }
}

mod variance {
    use cgmath::{Vector2, Point2, Point3, Vector3};
    use cgmath::prelude::*;

    use Real;

    pub trait Variance {
        type Point: EuclideanSpace<Scalar = Real>;

        fn new() -> Self;
        fn clear(&mut self);
        fn add_to_sum(&mut self, min: &Self::Point, max: &Self::Point);
        fn compute_axis(&self, n: Real) -> (usize, Real);
    }

    /// Variance for 2D sweep and prune
    #[derive(Debug)]
    pub struct Variance2 {
        csum: Vector2<Real>,
        csumsq: Vector2<Real>,
    }

    impl Variance for Variance2 {
        type Point = Point2<Real>;

        fn new() -> Self {
            Self {
                csum: Vector2::zero(),
                csumsq: Vector2::zero(),
            }
        }

        fn clear(&mut self) {
            self.csum = Vector2::zero();
            self.csumsq = Vector2::zero();
        }

        #[inline]
        fn add_to_sum(&mut self, min: &Point2<Real>, max: &Point2<Real>) {
            let min_vec = min.to_vec();
            let max_vec = max.to_vec();
            let sum = min_vec.add_element_wise(max_vec);
            let c = sum / 2.;
            self.csum.add_element_wise(c);
            self.csumsq.add_element_wise(c.mul_element_wise(c));
        }

        #[inline]
        fn compute_axis(&self, n: Real) -> (usize, Real) {
            let square_n = self.csum.mul_element_wise(self.csum) / n;
            let variance = self.csumsq.sub_element_wise(square_n);
            let mut sweep_axis = 0;
            let mut sweep_variance = variance[0];
            for i in 1..2 {
                let v = variance[i];
                if v > sweep_variance {
                    sweep_axis = i;
                    sweep_variance = v;
                }
            }
            (sweep_axis, sweep_variance)
        }
    }

    /// Variance for 3D sweep and prune
    #[derive(Debug)]
    pub struct Variance3 {
        csum: Vector3<Real>,
        csumsq: Vector3<Real>,
    }

    impl Variance for Variance3 {
        type Point = Point3<Real>;

        fn new() -> Self {
            Self {
                csum: Vector3::zero(),
                csumsq: Vector3::zero(),
            }
        }

        fn clear(&mut self) {
            self.csum = Vector3::zero();
            self.csumsq = Vector3::zero();
        }

        #[inline]
        fn add_to_sum(&mut self, min: &Point3<Real>, max: &Point3<Real>) {
            let min_vec = min.to_vec();
            let max_vec = max.to_vec();
            let sum = min_vec.add_element_wise(max_vec);
            let c = sum / 2.;
            self.csum.add_element_wise(c);
            self.csumsq.add_element_wise(c.mul_element_wise(c));
        }

        #[inline]
        fn compute_axis(&self, n: Real) -> (usize, Real) {
            let square_n = self.csum.mul_element_wise(self.csum) / n;
            let variance = self.csumsq.sub_element_wise(square_n);
            let mut sweep_axis = 0;
            let mut sweep_variance = variance[0];
            for i in 1..3 {
                let v = variance[i];
                if v > sweep_variance {
                    sweep_axis = i;
                    sweep_variance = v;
                }
            }
            (sweep_axis, sweep_variance)
        }
    }
}

#[cfg(test)]
mod tests {
    use cgmath::Point2;
    use collision::Aabb2;

    use super::*;
    use Real;
    use collide2d::{BroadCollisionInfo2, SweepAndPrune2};

    #[test]
    fn no_intersection_for_miss() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 12., 13., 18., 18.);

        let mut sweep = SweepAndPrune2::new();
        let potentials = sweep.compute(&mut vec![left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 12., 13., 18., 18.);

        let mut sweep = SweepAndPrune2::new();
        let potentials = sweep.compute(&mut vec![right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 9., 10., 18., 18.);

        let mut sweep = SweepAndPrune2::new();
        let potentials = sweep.compute(&mut vec![left, right]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 2), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(222, 9., 10., 18., 18.);

        let mut sweep = SweepAndPrune2::new();
        let potentials = sweep.compute(&mut vec![right, left]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 222), potentials[0]);
    }

    // util
    fn coll(
        id: u32,
        min_x: Real,
        min_y: Real,
        max_x: Real,
        max_y: Real,
    ) -> BroadCollisionInfo2<u32> {
        BroadCollisionInfo2::new(id, bound(min_x, min_y, max_x, max_y))
    }

    fn bound(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Aabb2<Real> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
