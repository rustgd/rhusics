use std::cmp::Ordering;
use two::collision::broad::*;
use collision::{Aabb2, Discrete};

use std::fmt::Debug;
use std::clone::Clone;

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

impl<ID: Debug + Clone> BroadPhase<ID> for SweepAndPrune {
    fn compute(&mut self, shapes: &mut Vec<BroadCollisionInfo<ID>>) -> Vec<(ID, ID)> {
        let mut pairs = Vec::<(ID, ID)>::default();
        if shapes.len() <= 1 {
            return pairs;
        }
        debug!("Starting sweep and prune");
        debug!("Sweep axis is {}", self.sweep_axis);
        shapes.sort_by(|a, b| if a.bound.min[self.sweep_axis] !=
            b.bound.min[self.sweep_axis]
        {
            a.bound.min[self.sweep_axis]
                .partial_cmp(&b.bound.min[self.sweep_axis])
                .unwrap_or(Ordering::Equal)
        } else {
            a.bound.max[self.sweep_axis]
                .partial_cmp(&b.bound.max[self.sweep_axis])
                .unwrap_or(Ordering::Equal)
        });
        debug!("Sorted vector {:?}", shapes);

        let mut active_index = 0;

        let mut csum = [0.; 2];
        let mut csumsq = [0.; 2];

        variance_sum(&mut csum, &mut csumsq, shapes[active_index].bound);
        debug!("starting checks");
        for index in 1..shapes.len() {
            debug!("before advance, active: {}, index: {}", active_index, index);
            // advance active_index until it could be intersecting
            while shapes[active_index].bound.max[self.sweep_axis] <
                shapes[index].bound.min[self.sweep_axis] && active_index < index
            {
                active_index += 1;
            }
            debug!("after advance, active: {}, index: {}", active_index, index);
            if index > active_index {
                for left_index in active_index..index {
                    if (shapes[left_index].bound, shapes[index].bound).intersects() {
                        pairs.push((shapes[left_index].id.clone(), shapes[index].id.clone()));
                    }
                }
            }
            variance_sum(&mut csum, &mut csumsq, shapes[index].bound);
        }

        let n = shapes.len() as f32;
        let mut sweep_variance = variance(csum[0], csumsq[0], n);
        self.sweep_axis = 0;
        for i in 1..2 {
            let v = variance(csum[i], csumsq[i], n);
            if v > sweep_variance {
                self.sweep_axis = i;
                sweep_variance = v;
            }
        }
        pairs
    }
}

#[inline]
fn variance(csum: f32, csumsq: f32, n: f32) -> f32 {
    csumsq - csum * csum / n
}

#[inline]
fn variance_sum(csum: &mut [f32; 2], csumsq: &mut [f32; 2], aabb: Aabb2<f32>) {
    for i in 0..2 {
        let c = 0.5 * (aabb.min[i] + aabb.max[i]);
        csum[i] += c;
        csumsq[i] += c * c;
    }
}

#[cfg(test)]
mod tests {
    use cgmath::Point2;
    use super::*;

    #[test]
    fn no_intersection_for_miss() {
        let left =
            BroadCollisionInfo::new_impl(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right = BroadCollisionInfo::new_impl(
            2,
            Aabb2::new(Point2::new(12., 13.), Point2::new(18., 18.)),
        );

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let left =
            BroadCollisionInfo::new_impl(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right = BroadCollisionInfo::new_impl(
            2,
            Aabb2::new(Point2::new(12., 13.), Point2::new(18., 18.)),
        );

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let left =
            BroadCollisionInfo::new_impl(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right = BroadCollisionInfo::new_impl(
            2,
            Aabb2::new(Point2::new(9., 10.), Point2::new(18., 18.)),
        );

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![left, right]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 2), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let left =
            BroadCollisionInfo::new_impl(1, Aabb2::new(Point2::new(8., 8.), Point2::new(10., 11.)));

        let right = BroadCollisionInfo::new_impl(
            222,
            Aabb2::new(Point2::new(9., 10.), Point2::new(18., 18.)),
        );

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![right, left]);
        assert_eq!(1, potentials.len());
        assert_eq!((1, 222), potentials[0]);
    }
}
