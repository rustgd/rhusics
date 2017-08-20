use specs::{System, SystemData, ReadStorage, WriteStorage};
use cgmath::prelude::*;
use cgmath::{BaseFloat};
use collision::{Aabb, MinMax, Discrete};

use std::fmt::Debug;

use collide::CollisionShape;

pub struct CollisionSystem {}

impl<'a, ID, S, V, P, R, A> System<'a> for CollisionSystem
    where
        ID: Clone + Debug,
        S: BaseFloat,
        V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
        P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
        R: Rotation<P>,
        A: Aabb<S, V, P> + Discrete<A>,
{
    type SystemData = ReadStorage<'a, CollisionShape<ID, S, V, P, R, A>>;

    fn run(&mut self, data: Self::SystemData) {}
}