use specs::{Component, VecStorage};
use cgmath::prelude::*;
use cgmath::BaseFloat;

use BodyPose;

impl<P, R> Component for BodyPose<P, R>
where
    P: EuclideanSpace + Send + Sync + 'static,
    P::Scalar: BaseFloat + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
{
    type Storage = VecStorage<BodyPose<P, R>>;
}
