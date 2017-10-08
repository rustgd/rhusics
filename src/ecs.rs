use cgmath::prelude::*;
use specs::{Component, DenseVecStorage, FlaggedStorage};

use {BodyPose, Real};

impl<P, R> Component for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
{
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}
