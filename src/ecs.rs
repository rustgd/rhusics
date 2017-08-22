use specs::{Component, VecStorage};
use cgmath::prelude::*;
use cgmath::BaseFloat;
use collision::MinMax;

use BodyPose;

impl<S, V, P, R> Component for BodyPose<S, V, P, R>
where
    S: BaseFloat + Send + Sync + 'static,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>
        + Send
        + Sync
        + 'static,
    P: EuclideanSpace<Scalar = S, Diff = V>
        + MinMax
        + Send
        + Sync
        + 'static,
    R: Rotation<P> + Send + Sync + 'static,
{
    type Storage = VecStorage<BodyPose<S, V, P, R>>;
}