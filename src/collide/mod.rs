pub mod broad;
pub mod narrow;
pub mod primitive2d;
pub mod primitive3d;
pub mod ecs;

use std::fmt::Debug;

use cgmath::prelude::*;
use collision::Aabb;

use {Pose, Real};

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
    A: Aabb<Scalar = Real>,
{
    fn get_far_point<P>(&self, direction: &A::Diff, transform: &P) -> A::Point
    where
        P: Pose<A::Point>;
    fn get_bound(&self) -> A;
}

#[derive(Debug)]
pub struct CollisionPrimitive<P, A, T>
where
    A: Aabb,
{
    local_transform: T,
    base_bound: A,
    transformed_bound: A,
    primitive: P,
}

impl<P, A, T> CollisionPrimitive<P, A, T>
where
    A: Aabb<Scalar = Real> + Clone,
    P: Primitive<A>,
    T: Pose<A::Point>,
{
    pub fn new(primitive: P) -> Self {
        Self::new_impl(primitive, T::one())
    }

    pub fn new_impl(primitive: P, local_transform: T) -> Self {
        let bound = primitive.get_bound().transform(&local_transform);
        Self {
            local_transform,
            base_bound: bound.clone(),
            transformed_bound: bound,
            primitive,
        }
    }

    pub fn update(&mut self, transform: &T) {
        self.transformed_bound = self.base_bound.transform(transform)
    }

    pub fn get_far_point(&self, direction: &A::Diff, transform: &T) -> A::Point {
        let t = transform.concat(&self.local_transform);
        self.primitive.get_far_point(direction, &t)
    }
}

#[derive(Debug)]
pub struct CollisionShape<P, A, T>
where
    A: Aabb,
    A::Diff: Debug,
{
    pub enabled: bool,
    base_bound: A,
    transformed_bound: A,
    primitives: Vec<CollisionPrimitive<P, A, T>>,
    strategy: CollisionStrategy,
}

impl<P, A, T> CollisionShape<P, A, T>
where
    A: Aabb<Scalar = Real> + Clone,
    A::Diff: Debug,
    P: Primitive<A>,
    T: Pose<A::Point>,
{
    pub fn new_complex(
        strategy: CollisionStrategy,
        primitives: Vec<CollisionPrimitive<P, A, T>>,
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

    pub fn new_simple_offset(strategy: CollisionStrategy, primitive: P, transform: T) -> Self {
        Self::new_complex(
            strategy,
            vec![CollisionPrimitive::new_impl(primitive, transform)],
        )
    }

    pub fn update(&mut self, transform: &T) {
        self.transformed_bound = self.base_bound.transform(transform);
        for mut primitive in &mut self.primitives {
            primitive.update(transform)
        }
    }
}

fn get_bound<P, A, T>(primitives: &Vec<CollisionPrimitive<P, A, T>>) -> A
where
    A: Aabb<Scalar = Real>,
    P: Primitive<A>,
{
    primitives.iter().map(|p| &p.base_bound).fold(
        A::zero(),
        |bound, b| {
            bound.union(b)
        },
    )
}
