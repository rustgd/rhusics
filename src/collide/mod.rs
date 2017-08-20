pub mod broad;
pub mod narrow;
pub mod primitive2d;
pub mod system;

use cgmath::prelude::*;
use cgmath::{Decomposed, BaseFloat};
use collision::{Aabb, Discrete, MinMax};

use std::fmt::Debug;

#[derive(Debug, PartialEq)]
pub enum CollisionStrategy {
    FullResolution,
    CollisionOnly,
}

#[derive(Debug)]
pub struct Contact<ID, S, V>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
{
    pub strategy: CollisionStrategy,
    pub bodies: (ID, ID),
    pub normal: V,
    pub penetration_depth: S,
}

impl <ID, S, V> Contact<ID, S, V> where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + Zero,
{
    pub fn new(strategy: CollisionStrategy, bodies: (ID, ID)) -> Self {
        Self::new_impl(strategy, bodies, V::zero(), S::zero())
    }

    pub fn new_impl(
        strategy: CollisionStrategy,
        bodies: (ID, ID),
        normal: V,
        penetration_depth: S,
    ) -> Self {
        Self {
            strategy,
            bodies,
            normal,
            penetration_depth,
        }
    }
}

pub trait Primitive<S, V, P, R, A>: Debug + Send + Sync
    where
        S: BaseFloat,
        V: VectorSpace<Scalar=S>
        + ElementWise
        + Array<Element=S>,
        P: EuclideanSpace<Scalar=S, Diff=V> + MinMax,
        R: Rotation<P>,
        A: Aabb<S, V, P> + Discrete<A>,
{
    fn get_far_point(&self,
                     direction: &V,
                     transform: &Decomposed<V, R>) -> P;
    fn get_bound(&self) -> A;
}

#[derive(Debug)]
pub struct CollisionPrimitive<S, V, P, R, A>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    R: Rotation<P>,
    A: Aabb<S, V, P> + Discrete<A>,
{
    local_transform: Decomposed<V, R>,
    base_bound: A,
    transformed_bound: A,
    primitive: Box<Primitive<S, V, P, R, A>>,
}

impl<S, V, P, R, A> CollisionPrimitive<S, V, P, R, A>
    where
        S: BaseFloat,
        V: VectorSpace<Scalar=S>
        + ElementWise
        + Array<Element=S>,
        P: EuclideanSpace<Scalar=S, Diff=V> + MinMax,
        R: Rotation<P>,
        A: Aabb<S, V, P> + Discrete<A> + Clone,
{
    pub fn new<PRIM: Primitive<S, V, P, R, A> + 'static>(primitive: PRIM) -> Self {
        Self::new_impl(primitive, Decomposed::one())
    }

    pub fn new_impl<PRIM: Primitive<S, V, P, R, A> + 'static>(
        primitive: PRIM,
        local_transform: Decomposed<V, R>
    ) -> Self {
        let bound = primitive.get_bound().transform(&local_transform);
        Self {
            local_transform,
            base_bound: bound.clone(),
            transformed_bound: bound,
            primitive: Box::new(primitive),
        }
    }

    pub fn update(&mut self, transform: &Decomposed<V, R>) {
        self.transformed_bound = self.base_bound.transform(transform)
    }

    pub fn get_far_point(&self, direction: &V, transform: &Decomposed<V, R>) -> P {
        let t = transform.concat(&self.local_transform);
        self.primitive.get_far_point(direction, &t)
    }
}

#[derive(Debug)]
pub struct CollisionShape<ID, S, V, P, R, A>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    R: Rotation<P>,
    A: Aabb<S, V, P> + Discrete<A>,
{
    pub id: ID,
    pub enabled: bool,
    pub base_bound: A,
    pub transformed_bound: A,
    pub primitives: Vec<CollisionPrimitive<S, V, P, R, A>>,
    pub strategy: CollisionStrategy,
}

impl<ID, S, V, P, R, A> CollisionShape<ID, S, V, P, R, A>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V>
        + MinMax
        + Zero,
    R: Rotation<P>,
    A: Aabb<S, V, P> + Clone + Discrete<A>,
{
    pub fn new(
        id: ID,
        strategy: CollisionStrategy,
        primitives: Vec<CollisionPrimitive<S, V, P, R, A>>,
    ) -> Self {
        let bound = get_bound(&primitives);
        Self {
            base_bound: bound.clone(),
            id,
            primitives,
            enabled: false,
            transformed_bound: bound,
            strategy,
        }
    }

    pub fn update(&mut self, transform: &Decomposed<V, R>) {
        self.transformed_bound = self.base_bound.transform(transform);
        for mut primitive in &mut self.primitives {
            primitive.update(transform)
        }
    }
}

fn get_bound<S, V, P, R, A>(primitives: &Vec<CollisionPrimitive<S, V, P, R, A>>) -> A
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax + Zero,
    R: Rotation<P>,
    A: Aabb<S, V, P> + Clone + Discrete<A>,
{
    primitives.iter().map(|p| &p.base_bound).fold(
        A::new(
            P::zero(),
            P::zero(),
        ),
        |bound, b| {
            bound.union(b)
        },
    )
}
