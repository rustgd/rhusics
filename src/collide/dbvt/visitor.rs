//! Concrete visitor implementations for use with
//! [`query`](struct.DynamicBoundingVolumeTree.html#method.query).
//!

use std::marker::PhantomData;

use collision::{Bound, Frustum, Relation};
use collision::prelude::*;

use super::{Visitor, TreeValue};
use Real;

/// Visitor for doing continuous intersection testing on the DBVT.
///
/// Will return the result from the
/// [`Continuous`](https://docs.rs/collision/0.11.0/collision/trait.Continuous.html) implementation
/// of bound.intersection(self.bound).
///
#[derive(Debug)]
pub struct ContinuousVisitor<'a, B: 'a, T> {
    bound: &'a B,
    marker: PhantomData<T>,
}

impl<'a, B: 'a, T> ContinuousVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Continuous<B> + Discrete<B>,
{
    /// Create a new visitor that will do continuous intersection tests using the given bound.
    ///
    pub fn new(bound: &'a B) -> Self {
        Self {
            bound,
            marker: PhantomData,
        }
    }
}

impl<'a, B: 'a, T> Visitor for ContinuousVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Continuous<B> + Discrete<B>,
{
    type Bound = T::Bound;
    type Result = <T::Bound as Continuous<B>>::Result;

    fn accept(&self, bound: &Self::Bound) -> Option<Self::Result> {
        bound.intersection(self.bound)
    }
}

/// Visitor for doing discrete intersection testing on the DBVT.
///
/// Will return () for intersections with the
/// [`Discrete`](https://docs.rs/collision/0.11.0/collision/trait.Discrete.html) implementation
/// of bound.intersects(self.bound).
///
#[derive(Debug)]
pub struct DiscreteVisitor<'a, B: 'a, T> {
    bound: &'a B,
    marker: PhantomData<T>,
}

impl<'a, B: 'a, T> DiscreteVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Discrete<B>,
{
    /// Create a new visitor that will do discrete intersection tests using the given bound.
    pub fn new(bound: &'a B) -> Self {
        Self {
            bound,
            marker: PhantomData,
        }
    }
}

impl<'a, B: 'a, T> Visitor for DiscreteVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Discrete<B>,
{
    type Bound = T::Bound;
    type Result = ();

    fn accept(&self, bound: &Self::Bound) -> Option<()> {
        if bound.intersects(self.bound) {
            Some(())
        } else {
            None
        }
    }
}

/// Visitor for doing frustum intersection testing on the DBVT.
///
/// Will return the relation for intersections with the
/// [`Bound`](https://docs.rs/collision/0.11.0/collision/trait.Bound.html) implementation
/// of self.frustum.contains(bound).
///
#[derive(Debug)]
pub struct FrustumVisitor<'a, T> {
    frustum: &'a Frustum<Real>,
    marker: PhantomData<T>,
}

impl<'a, T> FrustumVisitor<'a, T>
where
    T: TreeValue,
    T::Bound: Bound<Real>,
{
    /// Create a new visitor that will do containment tests using the given frustum
    pub fn new(frustum: &'a Frustum<Real>) -> Self {
        Self {
            frustum,
            marker: PhantomData,
        }
    }
}

impl<'a, T> Visitor for FrustumVisitor<'a, T>
where
    T: TreeValue,
    T::Bound: Bound<Real>,
{
    type Bound = T::Bound;
    type Result = Relation;

    fn accept(&self, bound: &Self::Bound) -> Option<Relation> {
        let r = self.frustum.contains(bound);
        if r == Relation::Out { None } else { Some(r) }
    }
}
