pub mod simplex;
pub mod epa;

use super::NarrowPhase;

use collide::{Contact, ContactSet, CollisionShape, CollisionStrategy, CollisionPrimitive};

use cgmath::prelude::*;
use cgmath::{BaseFloat, Decomposed};
use collision::{Aabb, Discrete, MinMax};

use std;
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

impl<S, V, P> GJK<S, V, P>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: SimplexProcessor<S, V>,
{
    pub fn new() -> Self {
        Self {
            simplex_processor: P::new(),
            marker: std::marker::PhantomData,
        }
    }
}

impl<ID, S, V, P, A, R, SP> NarrowPhase<ID, S, V, P, R, A> for GJK<S, V, SP>
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
        + MinMax + Debug,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
    SP: SimplexProcessor<S, V>
{
    fn collide(
        &mut self,
        (ref left_id, ref mut left, ref left_transform):
            (ID, &CollisionShape<S, V, P, R, A>, &Decomposed<V, R>),
        (ref right_id, ref mut right, ref right_transform):
            (ID, &CollisionShape<S, V, P, R, A>, &Decomposed<V, R>),
    ) -> Option<ContactSet<ID, S, V>> {
        let mut contacts = Vec::default();
        if !left.enabled || !right.enabled || left.primitives.is_empty() ||
            right.primitives.is_empty()
        {
            return None;
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
                                contacts.push((Contact::new(CollisionStrategy::CollisionOnly)));
                            } else {
                                contacts.append(&mut self::epa::epa(&mut simplex,
                                                              &left_primitive,
                                                                    left_transform,
                                                             &right_primitive,
                                                             right_transform));
                            }
                        }
                        None => (),
                    };
                }
            }
        }

        if contacts.len() > 0 {
            Some(ContactSet::new((left_id.clone(), right_id.clone()), contacts))
        } else {
            None
        }
    }
}

fn gjk<S, V, P, A, R, SP>(
    left: &CollisionPrimitive<S, V, P, R, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, R, A>,
    right_transform: &Decomposed<V, R>,
    simplex_processor: &SP,
) -> Option<Vec<V>>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace + Neg<Output = V>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax + Debug,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
    SP: SimplexProcessor<S, V>,
{
    let mut d = right_transform.disp - left_transform.disp;
    let a = support(left, left_transform, right, right_transform, &d);
    if a.dot(d) <= S::zero() {
        return None;
    }
    let mut simplex: Vec<V> = Vec::default();
    simplex.push(a);
    d = d.neg();
    let mut i = 0;
    loop {
        let a = support(left, left_transform, right, right_transform, &d);
        if a.dot(d) <= S::zero() {
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
    left: &CollisionPrimitive<S, V, P, R, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, R, A>,
    right_transform: &Decomposed<V, R>,
    direction: &V,
) -> V
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + Neg<Output = V>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax + Debug,
    A: Aabb<S, V, P> + Discrete<A>,
    R: Rotation<P>,
{
    let l = left.primitive.get_far_point(direction, left_transform);
    let r = right.primitive.get_far_point(
        &direction.neg(),
        right_transform,
    );
    l - r
}

#[cfg(test)]
mod tests {
    use super::GJK;
    use super::support;
    use cgmath::{Vector2, Decomposed, Rotation2, Rad, Basis2};
    use collide2d::*;

    #[test]
    fn test_support() {
        let left = CollisionPrimitive2D::<f32>::new(Rectangle::new(10., 10.));
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionPrimitive2D::<f32>::new(Rectangle::new(10., 10.));
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(-15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let direction = Vector2::new(1., 0.);
        assert_eq!(
            Vector2::new(40., 0.),
            support(&left, &left_transform, &right, &right_transform, &direction)
        );
    }

}
