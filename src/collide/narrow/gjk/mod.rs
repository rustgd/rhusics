pub mod simplex;
pub mod epa;

use super::NarrowPhase;

use collide::{Contact, CollisionShape, CollisionStrategy, CollisionPrimitive};

use cgmath;
use cgmath::{BaseFloat, VectorSpace, ElementWise, Array, EuclideanSpace, Transform, Decomposed,
             Rotation, MetricSpace, InnerSpace};
use collision::{Aabb, Discrete, MinMax};

use std;
use std::cmp::Ordering;
use std::ops::Neg;
use std::fmt::Debug;

use self::simplex::SimplexProcessor;

const MAX_ITERATIONS: usize = 100;

pub struct GJK<S, V, P>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: SimplexProcessor<S, V>,
{
    simplex_processor: P,
    marker: std::marker::PhantomData<(S, V)>,
}

impl<ID, S, V, P, A, R, SP> NarrowPhase<ID, S, V, P, Decomposed<V, R>, A, R> for GJK<S, V, SP>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>
        + MetricSpace
        + InnerSpace
        + Neg<Output = V>,
    P: EuclideanSpace<
        Scalar = S,
        Diff = V,
    >
        + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
    SP: SimplexProcessor<S, V>
{
    fn collide(
        &mut self,
        &mut (ref mut left, ref left_transform): &mut (CollisionShape<ID, S, V, P, Decomposed<V, R>, A>,
                                                       Decomposed<V, R>),
        &mut (ref mut right, ref right_transform): &mut (CollisionShape<ID, S, V, P, Decomposed<V, R>, A>,
                                                         Decomposed<V, R>),
    ) -> Option<Contact<ID, S, V>> {
        if !left.enabled || !right.enabled || left.primitives.is_empty() ||
            right.primitives.is_empty()
        {
            return None;
        }

        let v_zero = V::zero();
        if left.primitives.len() > 1 {
            left.primitives.sort_by(|a, b| {
                right_transform
                    .disp
                    .distance2(
                        left_transform.disp + a.local_transform.transform_vector(v_zero),
                    )
                    .partial_cmp(&right_transform.disp.distance2(
                        left_transform.disp +
                            b.local_transform
                                .transform_vector(v_zero),
                    ))
                    .unwrap_or(Ordering::Equal)
            });
        }
        if right.primitives.len() > 1 {
            right.primitives.sort_by(|a, b| {
                left_transform
                    .disp
                    .distance2(
                        right_transform.disp + a.local_transform.transform_vector(v_zero),
                    )
                    .partial_cmp(&left_transform.disp.distance2(
                        right_transform.disp +
                            b.local_transform.transform_vector(
                                v_zero,
                            ),
                    ))
                    .unwrap_or(Ordering::Equal)
            });
        }

        for left_primitive in &left.primitives {
            for right_primitive in &right.primitives {
                if left.transformed_bound.intersects(&right.transformed_bound) {
                    match gjk(
                        &left_primitive,
                        left_transform,
                        &right_primitive,
                        right_transform,
                        &self.simplex_processor
                    ) {
                        Some(mut simplex) => {
                            if left.strategy == CollisionStrategy::CollisionOnly ||
                                right.strategy == CollisionStrategy::CollisionOnly
                            {
                                return Some(Contact::new(CollisionStrategy::CollisionOnly, (
                                    left.id.clone(),
                                    right.id.clone(),
                                )));
                            } else {
                                return Some(self::epa::epa((left.id.clone(), right.id.clone()),
                                                            &mut simplex,
                                                            &left_primitive,
                                                            left_transform,
                                                            &right_primitive,
                                                            right_transform)); // TODO: EPA
                            }
                        }
                        None => (),
                    };
                }
            }
        }

        None
    }
}

fn gjk<S, V, P, A, R, SP>(
    left: &CollisionPrimitive<S, V, P, Decomposed<V, R>, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, Decomposed<V, R>, A>,
    right_transform: &Decomposed<V, R>,
    simplex_processor: &SP,
) -> Option<Vec<V>>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace + Neg<Output = V>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
    SP: SimplexProcessor<S, V>,
{
    let mut d = right_transform.disp - left_transform.disp;
    let mut simplex: Vec<V> = Vec::default();
    simplex.push(support(left, left_transform, right, right_transform, &d));
    if cgmath::dot(*simplex.last().unwrap(), d) <= S::zero() {
        return None;
    }
    d = d.neg();
    let mut i = 0;
    loop {
        let a = support(left, left_transform, right, right_transform, &d);
        if cgmath::dot(a, d) <= S::zero() {
            return None;
        } else {
            simplex.push(a);
            if simplex_processor.process(&mut simplex, &mut d) {
                return Some(simplex);
            }
        }
        i += 1;
        if i >= MAX_ITERATIONS {
            return None;
        }
    }
}

fn support<S, V, P, A, R>(
    left: &CollisionPrimitive<S, V, P, Decomposed<V, R>, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, Decomposed<V, R>, A>,
    right_transform: &Decomposed<V, R>,
    direction: &V,
) -> V
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + Neg<Output = V>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
{
    left.primitive.get_far_point(direction, left_transform) -
        right.primitive.get_far_point(
            &direction.neg(),
            right_transform,
        )
}
