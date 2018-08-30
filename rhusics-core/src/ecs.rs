//! ECS Component declarations for data structures in the crate, this needs to be here and not in
//! rhusics-ecs because of the orphan rule.

use cgmath::prelude::*;
use cgmath::BaseFloat;
use collision::prelude::*;
use specs::prelude::{Component, DenseVecStorage, FlaggedStorage};

use collide::CollisionShape;
use physics::{ForceAccumulator, Mass, PhysicalEntity, Velocity};
use {BodyPose, NextFrame};

impl<P, R> Component for BodyPose<P, R>
where
    P: EuclideanSpace + Send + Sync + 'static,
    P::Scalar: BaseFloat,
    R: Rotation<P> + Send + Sync + 'static,
{
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<T> Component for NextFrame<T>
where
    T: Send + Sync + 'static,
{
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<P, T, B, Y> Component for CollisionShape<P, T, B, Y>
where
    T: Send + Sync + 'static,
    Y: Send + Sync + 'static,
    P: Primitive + Send + Sync + 'static,
    B: Bound + Send + Sync + 'static,
{
    type Storage = DenseVecStorage<CollisionShape<P, T, B, Y>>;
}

impl<V, A> Component for Velocity<V, A>
where
    V: Send + Sync + 'static + Clone,
    A: Send + Sync + 'static + Clone,
{
    type Storage = DenseVecStorage<Self>;
}

impl<S, I> Component for Mass<S, I>
where
    S: Send + Sync + 'static,
    I: Send + Sync + 'static,
{
    type Storage = DenseVecStorage<Self>;
}

impl<S> Component for PhysicalEntity<S>
where
    S: Send + Sync + 'static,
{
    type Storage = DenseVecStorage<Self>;
}

impl<F, A> Component for ForceAccumulator<F, A>
where
    F: Send + Sync + 'static,
    A: Send + Sync + 'static,
{
    type Storage = DenseVecStorage<Self>;
}
