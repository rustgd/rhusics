use cgmath::prelude::*;
use cgmath::{BaseFloat, Vector2, Vector3};
use std::ops::Neg;

pub trait SimplexProcessor<S, V>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>
        + InnerSpace,
{
    fn process(&self, simplex: &mut Vec<V>, d: &mut V) -> bool;
    fn new() -> Self;
}

pub struct SimplexProcessor2D;
pub struct SimplexProcessor3D;

impl<S> SimplexProcessor<S, Vector2<S>> for SimplexProcessor2D
where
    S: BaseFloat,
{
    fn process(&self, simplex: &mut Vec<Vector2<S>>, d: &mut Vector2<S>) -> bool {
        // 3 points
        if simplex.len() == 3 {
            let a = simplex[2];
            let ao = a.neg();
            let b = simplex[1];
            let c = simplex[0];
            let ab = b - a;
            let ac = c - a;
            let ab_perp = ::util::triple_product(&ac, &ab, &ab);
            if ab_perp.dot(ao) > S::zero() {
                simplex.remove(0);
                *d = ab_perp;
            } else {
                let ac_perp = ::util::triple_product(&ab, &ac, &ac);
                if ac_perp.dot(ao) > S::zero() {
                    simplex.remove(1);
                    *d = ac_perp;
                } else {
                    return true;
                }
            }
        }
        // 2 points
        else {
            let a = simplex[1];
            let ao = a.neg();
            let b = simplex[0];
            let ab = b - a;
            *d = ::util::triple_product(&ab, &ao, &ab);
        }
        // 0-1 point means we can't really do anything
        false
    }

    fn new() -> Self {
        Self {}
    }
}

impl<S> SimplexProcessor<S, Vector3<S>> for SimplexProcessor3D
where
    S: BaseFloat,
{
    fn  process(&self, simplex: &mut Vec<Vector3<S>>, d: &mut Vector3<S>) -> bool {
        // 4 points
        if simplex.len() == 4 {
            //TODO
        }
        // 3 points
        else if simplex.len() == 3 {
            //TODO
        }
        // 2 points
        else if simplex.len() == 3 {
            let a = simplex[1];
            let ao = a.neg();
            let b = simplex[0];
            let ab = b - a;
            if ab.dot(ao) > S::zero() {
                *d = ab.cross(ao).cross(ab);
            } else {
                simplex.remove(0);
                *d = ao;
            }
        }
        // 0-1 points
        false
    }

    fn new() -> Self {
        Self {}
    }
}
