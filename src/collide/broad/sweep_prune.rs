use std::cmp::Ordering;
use collide::broad::*;

use std::fmt::Debug;
use std::clone::Clone;
use std::ops::{RangeFull, Index};

#[derive(Debug)]
pub struct SweepAndPrune {
    sweep_axis: usize,
}

impl SweepAndPrune {
    pub fn new() -> SweepAndPrune {
        Self::new_impl(0)
    }

    pub fn new_impl(sweep_axis: usize) -> SweepAndPrune {
        SweepAndPrune { sweep_axis }
    }
}

impl<ID, S, V, P, A> BroadPhase<ID, S, V, P, A> for SweepAndPrune where
    ID: Clone + Debug,
    S: BaseFloat + Debug,
    V: VectorSpace<Scalar=S>
        + ElementWise
        + Array<Element=S>
        + Index<RangeFull, Output = [S]>
        + Debug,
    P: EuclideanSpace<Scalar=S, Diff=V> + MinMax + Debug,
    A: Aabb<S, V, P> + Discrete<A> + Debug,
{
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID, S, V, P, A>>) -> Vec<(ID, ID)> {
        let mut pairs = Vec::<(ID, ID)>::default();
        if shapes.len() <= 1 {
            return pairs;
        }
        debug!("Starting sweep and prune");
        debug!("Sweep axis is {}", self.sweep_axis);
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
        debug!("Sorted vector {:?}", shapes);

        let mut active_index = 0;

        let mut csum = V::zero();
        let mut csumsq = V::zero();

        variance_sum(&mut csum, &mut csumsq, &shapes[active_index].bound);
        debug!("starting checks");
        for index in 1..shapes.len() {
            debug!("before advance, active: {}, index: {}", active_index, index);
// advance active_index until it could be intersecting
            while shapes[active_index].bound.max()[self.sweep_axis] <
                shapes[index].bound.min()[self.sweep_axis] && active_index < index
                {
                    active_index += 1;
                }
            debug!("after advance, active: {}, index: {}", active_index, index);
            if index > active_index {
                for left_index in active_index..index {
                    if shapes[left_index].bound.intersects(&shapes[index].bound) {
                        pairs.push((shapes[left_index].id.clone(), shapes[index].id.clone()));
                    }
                }
            }
            variance_sum(&mut csum, &mut csumsq, &shapes[index].bound);
        }

        let n = S::from(shapes.len()).unwrap();
        let variance = csumsq.sub_element_wise(csum.mul_element_wise(csum) / n);
        let len = variance[..].len();
        self.sweep_axis = 0;
        let mut sweep_variance = S::neg_infinity();
        for i in 0..len {
            let v = variance[i];
            if v > sweep_variance {
                self.sweep_axis = i;
                sweep_variance = v;
            }
        }
        pairs
    }
}

#[inline]
fn variance_sum<S, V, P, A>(csum: &mut V, csumsq: &mut V, aabb: &A)
where
    S: BaseFloat + Debug,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + Debug,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax + Debug,
    A: Aabb<S, V, P> + Debug,
{
    let two = S::one() + S::one();
    let min = aabb.min();
    let max = aabb.max();
    let c = min.to_vec().add_element_wise(max.to_vec()) / two;
    csum.add_element_wise(c);
    csumsq.add_element_wise(c.mul_element_wise(c));
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

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let left =
            BroadCollisionInfo::new(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right =
            BroadCollisionInfo::new(2, Aabb2::new(Point2::new(12., 13.), Point2::new(18., 18.)));

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let left =
            BroadCollisionInfo::new(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right =
            BroadCollisionInfo::new(2, Aabb2::new(Point2::new(9., 10.), Point2::new(18., 18.)));

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![left, right]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 2), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let left =
            BroadCollisionInfo::new(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right =
            BroadCollisionInfo::new(222, Aabb2::new(Point2::new(9., 10.), Point2::new(18., 18.)));

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![right, left]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 222), potentials[0]);
    }
}
