use cgmath::BaseFloat;
use cgmath::prelude::*;
use collision::prelude::*;
use specs::{Component, DenseVecStorage, FlaggedStorage};

use {BodyPose, NextFrame};
use collide::CollisionShape;

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
