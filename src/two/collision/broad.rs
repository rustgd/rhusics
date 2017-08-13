use std::cmp::Ordering;
pub use super::CollisionShape;
use super::super::BodyPose;
use collision::{Aabb2, Discrete};

pub trait BroadPhase {
    fn compute(&mut self, shapes: &mut Vec<(CollisionShape, BodyPose)>) -> Vec<(usize, usize)>;
}

#[derive(Debug)]
pub struct SweepAndPrune {
    sweep_axis : usize
}

impl SweepAndPrune {
    pub fn new() -> SweepAndPrune {
        Self::new_impl(0)
    }

    pub fn new_impl(sweep_axis : usize) -> SweepAndPrune {
        SweepAndPrune {
            sweep_axis
        }
    }
}

impl BroadPhase for SweepAndPrune {
    fn compute(&mut self, shapes: &mut Vec<(CollisionShape, BodyPose)>) -> Vec<(usize, usize)> {
        let mut pairs = Vec::<(usize, usize)>::default();
        if shapes.len() <= 1 {
            return pairs;
        }
        debug!("Starting sweep and prune");
        debug!("Sweep axis is {}", self.sweep_axis);
        shapes.sort_by(|&(ref a, _), &(ref b, _)| {
            if a.transformed_bound.min[self.sweep_axis] != b.transformed_bound.min[self.sweep_axis] {
                a.transformed_bound.min[self.sweep_axis]
                    .partial_cmp(&b.transformed_bound.min[self.sweep_axis])
                    .unwrap_or(Ordering::Equal)
            } else {
                a.transformed_bound.max[self.sweep_axis]
                    .partial_cmp(&b.transformed_bound.max[self.sweep_axis])
                    .unwrap_or(Ordering::Equal)
            }
        });
        debug!("Sorted vector {:?}", shapes);

        let mut active_index = 0;

        let mut csum = [0.; 2];
        let mut csumsq = [0.; 2];

        variance_sum(&mut csum, &mut csumsq, shapes[active_index].0.transformed_bound);
        debug!("starting checks");
        for index in 1..shapes.len() {
            if !shapes[index].0.enabled {
                continue;
            }
            debug!("before advance, active: {}, index: {}", active_index, index);
            // advance active_index until it could be intersecting
            while (!shapes[active_index].0.enabled
                || shapes[active_index].0.transformed_bound.max[self.sweep_axis] <
                   shapes[index].0.transformed_bound.min[self.sweep_axis])
                && active_index < index {
                active_index += 1;
            }
            debug!("after advance, active: {}, index: {}", active_index, index);
            if index > active_index {
                for left_index in active_index..index {
                    if shapes[left_index].0.enabled &&
                        (shapes[left_index].0.transformed_bound,
                         shapes[index].0.transformed_bound).intersects() {
                        pairs.push((left_index, index));
                    }
                }
            }
            variance_sum(&mut csum, &mut csumsq, shapes[index].0.transformed_bound);
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
fn variance(csum : f32, csumsq : f32, n : f32) -> f32 {
    csumsq - csum * csum / n
}

#[inline]
fn variance_sum(csum : &mut [f32; 2], csumsq : &mut [f32; 2], aabb : Aabb2<f32>) {
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
        let mut left = CollisionShape::new(1, Vec::default());
        left.transformed_bound.min = Point2::new(8., 8.);
        left.transformed_bound.max = Point2::new(10., 11.);

        let mut right = CollisionShape::new(2, Vec::default());
        right.transformed_bound.min = Point2::new(12., 13.);
        right.transformed_bound.max = Point2::new(18., 18.);

        left.enabled = true;
        right.enabled = true;

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let mut left = CollisionShape::new(1, Vec::default());
        left.transformed_bound.min = Point2::new(8., 8.);
        left.transformed_bound.max = Point2::new(10., 11.);

        let mut right = CollisionShape::new(2, Vec::default());
        right.transformed_bound.min = Point2::new(12., 13.);
        right.transformed_bound.max = Point2::new(18., 18.);

        left.enabled = true;
        right.enabled = true;

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let mut left = CollisionShape::new(1, Vec::default());
        left.transformed_bound.min = Point2::new(8., 8.);
        left.transformed_bound.max = Point2::new(10., 11.);

        let mut right = CollisionShape::new(2, Vec::default());
        right.transformed_bound.min = Point2::new(9., 10.);
        right.transformed_bound.max = Point2::new(18., 18.);

        left.enabled = true;
        right.enabled = true;

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![left, right]);
        assert_eq!(1, potentials.len());
        assert_eq!((0, 1), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let mut left = CollisionShape::new(23, Vec::default());
        left.transformed_bound.min = Point2::new(8., 8.);
        left.transformed_bound.max = Point2::new(10., 11.);

        let mut right = CollisionShape::new(245, Vec::default());
        right.transformed_bound.min = Point2::new(9., 10.);
        right.transformed_bound.max = Point2::new(18., 18.);

        left.enabled = true;
        right.enabled = true;

        let mut sweep = SweepAndPrune::new();
        let potentials = sweep.compute(&mut vec![right, left]);
        assert_eq!(1, potentials.len());
        assert_eq!((0, 1), potentials[0]);
    }
}