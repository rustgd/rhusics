use cgmath::prelude::*;
use specs::{Component, VecStorage};

use {BodyPose, Real};

impl<P, R> Component for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
{
    type Storage = VecStorage<BodyPose<P, R>>;
}
