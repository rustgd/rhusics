//! Utility structures for use with collision detection systems

use std::fmt::Debug;

use cgmath::prelude::*;
use collision::{Aabb, Primitive};
use collision::algorithm::broad_phase::HasBound;
use collision::dbvt::TreeValue;

use Real;

/// Shape wrapper for use with containers such as DBVT, or for use with broad phase algorithms
#[derive(Debug, Clone)]
pub struct ContainerShapeWrapper<ID, P>
where
    P: Primitive,
    P::Aabb: Aabb<Scalar = Real>,
    <P::Point as EuclideanSpace>::Diff: Debug,
{
    /// The id
    pub id: ID,

    /// The bounding volume
    pub bound: P::Aabb,
    fat_factor: <P::Point as EuclideanSpace>::Diff,
}

impl<ID, P> ContainerShapeWrapper<ID, P>
where
    P: Primitive,
    P::Aabb: Clone + Aabb<Scalar = Real>,
    <P::Point as EuclideanSpace>::Diff: Debug,
{
    /// Create a new shape
    pub fn new_impl(
        id: ID,
        bound: &P::Aabb,
        fat_factor: <P::Point as EuclideanSpace>::Diff,
    ) -> Self {
        Self {
            id,
            bound: bound.clone(),
            fat_factor,
        }
    }

    /// Create a new shape
    pub fn new(id: ID, bound: &P::Aabb) -> Self {
        Self::new_impl(
            id,
            bound,
            <P::Point as EuclideanSpace>::Diff::from_value(Real::one()),
        )
    }
}

impl<ID, P> TreeValue for ContainerShapeWrapper<ID, P>
where
    ID: Clone + Debug,
    P: Primitive,
    P::Aabb: Aabb<Scalar = Real>,
    P::Point: Debug,
    <P::Point as EuclideanSpace>::Diff: Debug,
{
    type Bound = P::Aabb;

    fn bound(&self) -> &Self::Bound {
        &self.bound
    }

    fn fat_bound(&self) -> Self::Bound {
        self.bound.add_margin(self.fat_factor)
    }
}

impl<ID, P> HasBound for ContainerShapeWrapper<ID, P>
where
    P: Primitive,
    P::Aabb: Aabb<Scalar = Real>,
    <P::Point as EuclideanSpace>::Diff: Debug,
{
    type Bound = P::Aabb;

    fn get_bound(&self) -> &P::Aabb {
        &self.bound
    }
}
