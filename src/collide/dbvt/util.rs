//! Utility functions for use with
//! [`DynamicBoundingVolumeTree`](struct.DynamicBoundingVolumeTree.html).
//!

use std::fmt::Debug;

use cgmath::num_traits::Float;
use cgmath::prelude::*;
use collision::Ray;
use collision::prelude::*;

use super::{DynamicBoundingVolumeTree, TreeValue};
use super::visitor::ContinuousVisitor;
use Real;

/// Query the given tree for the closest value that intersects the given ray.
pub fn query_ray_closest<'a, T: 'a, P>(
    tree: &'a DynamicBoundingVolumeTree<T>,
    ray: &Ray<Real, P, P::Diff>,
) -> Option<(&'a T, P)>
where
    T: TreeValue,
    P: EuclideanSpace<Scalar = Real>,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace,
    T::Bound: Clone
        + Debug
        + Contains<T::Bound>
        + SurfaceArea<Real>
        + Union<T::Bound, Output = T::Bound>
        + Continuous<Ray<Real, P, P::Diff>, Result = P>
        + Discrete<Ray<Real, P, P::Diff>>,
{
    let mut saved = None;
    let mut tmin = Real::infinity();
    let visitor = ContinuousVisitor::<Ray<Real, P, P::Diff>, T>::new(&ray);
    for (value, point) in tree.query(&visitor) {
        let offset = point - ray.origin;
        let t = offset.dot(ray.direction);
        if t < tmin {
            tmin = t;
            saved = Some((value, point.clone()));
        }
    }
    saved
}
