pub mod broad;
pub mod narrow;
pub mod primitive2d;
pub mod ecs;

use cgmath::prelude::*;
use cgmath::{Decomposed, BaseFloat};
use collision::Aabb;

use std::fmt::Debug;

#[derive(Debug, PartialEq)]
pub enum CollisionStrategy {
    FullResolution,
    CollisionOnly,
}

#[derive(Debug)]
pub struct ContactSet<ID, V: VectorSpace> {
    pub bodies: (ID, ID),
    pub contacts: Vec<Contact<V>>,
}

impl<ID, V> ContactSet<ID, V>
where
    ID: Clone + Debug,
    V: VectorSpace + ElementWise + Array + Zero,
{
    pub fn new(bodies: (ID, ID), contacts: Vec<Contact<V>>) -> Self {
        Self { bodies, contacts }
    }

    pub fn new_single(strategy: CollisionStrategy, bodies: (ID, ID)) -> Self {
        Self::new(bodies, vec![Contact::new(strategy)])
    }
}

#[derive(Debug)]
pub struct Contact<V: VectorSpace> {
    pub strategy: CollisionStrategy,
    pub normal: V,
    pub penetration_depth: V::Scalar,
}

impl<V> Contact<V>
where
    V: VectorSpace + Zero,
{
    pub fn new(strategy: CollisionStrategy) -> Self {
        Self::new_impl(strategy, V::zero(), V::Scalar::zero())
    }

    pub fn new_impl(strategy: CollisionStrategy, normal: V, penetration_depth: V::Scalar) -> Self {
        Self {
            strategy,
            normal,
            penetration_depth,
        }
    }
}

pub trait Primitive<A>: Debug + Send + Sync
where
    A: Aabb,
    A::Scalar: BaseFloat,
{
    fn get_far_point<R: Rotation<A::Point>>(
        &self,
        direction: &A::Diff,
        transform: &Decomposed<A::Diff, R>,
    ) -> A::Point;
    fn get_bound(&self) -> A;
}

#[derive(Debug)]
pub struct CollisionPrimitive<P, A, R>
where
    A: Aabb,
{
    local_transform: Decomposed<A::Diff, R>,
    base_bound: A,
    transformed_bound: A,
    primitive: P,
}

impl<P, A, R> CollisionPrimitive<P, A, R>
where
    A: Aabb + Clone,
    A::Scalar: BaseFloat,
    P: Primitive<A>,
    R: Rotation<A::Point>,
{
    pub fn new(primitive: P) -> Self {
        Self::new_impl(primitive, Decomposed::one())
    }

    pub fn new_impl(primitive: P, local_transform: Decomposed<A::Diff, R>) -> Self {
        let bound = primitive.get_bound().transform(&local_transform);
        Self {
            local_transform,
            base_bound: bound.clone(),
            transformed_bound: bound,
            primitive,
        }
    }

    pub fn update(&mut self, transform: &Decomposed<A::Diff, R>) {
        self.transformed_bound = self.base_bound.transform(transform)
    }

    pub fn get_far_point(
        &self,
        direction: &A::Diff,
        transform: &Decomposed<A::Diff, R>,
    ) -> A::Point {
        let t = transform.concat(&self.local_transform);
        self.primitive.get_far_point(direction, &t)
    }
}

#[derive(Debug)]
pub struct CollisionShape<P, A, R>
where
    A: Aabb,
    A::Diff: Debug,
{
    pub enabled: bool,
    base_bound: A,
    transformed_bound: A,
    primitives: Vec<CollisionPrimitive<P, A, R>>,
    strategy: CollisionStrategy,
}

impl<P, A, R> CollisionShape<P, A, R>
where
    A: Aabb + Clone,
    A::Scalar: BaseFloat,
    A::Diff: Debug,
    P: Primitive<A>,
    R: Rotation<A::Point>,
{
    pub fn new_complex(
        strategy: CollisionStrategy,
        primitives: Vec<CollisionPrimitive<P, A, R>>,
    ) -> Self {
        let bound = get_bound(&primitives);
        Self {
            base_bound: bound.clone(),
            primitives,
            enabled: true,
            transformed_bound: bound,
            strategy,
        }
    }

    pub fn new_simple(strategy: CollisionStrategy, primitive: P) -> Self {
        Self::new_complex(strategy, vec![CollisionPrimitive::new(primitive)])
    }

    pub fn new_simple_offset(
        strategy: CollisionStrategy,
        primitive: P,
        transform: Decomposed<A::Diff, R>,
    ) -> Self {
        Self::new_complex(
            strategy,
            vec![CollisionPrimitive::new_impl(primitive, transform)],
        )
    }

    pub fn update(&mut self, transform: &Decomposed<A::Diff, R>) {
        self.transformed_bound = self.base_bound.transform(transform);
        for mut primitive in &mut self.primitives {
            primitive.update(transform)
        }
    }
}

fn get_bound<P, A, R>(primitives: &Vec<CollisionPrimitive<P, A, R>>) -> A
where
    A: Aabb,
    A::Scalar: BaseFloat,
    P: Primitive<A>,
{
    primitives.iter().map(|p| &p.base_bound).fold(
        A::zero(),
        |bound, b| {
            bound.union(b)
        },
    )
}
