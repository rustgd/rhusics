use std::fmt::Debug;
use std::marker;
use std::ops::Mul;

use cgmath::{BaseFloat, EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};
use specs::{Fetch, Join, ReadStorage, System, WriteStorage};

use {BodyPose, NextFrame};
use ecs::physics::resources::DeltaTime;
use physics::{ApplyAngular, ForceAccumulator, Inertia, Mass, Velocity};
use physics::simple::*;

/// Setup the next frames positions and velocities.
///
/// ### Type parameters:
///
/// - `P`: Positional quantity, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
/// - `I`: Inertia, usually `Scalar` or `Matrix3`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
///
/// ### System function
///
/// `fn(DeltaTime, Mass, BodyPose, ForceAccumulator) -> (ForceAccumulator, NextFrame<Velocity>, NextFrame<BodyPose>)`
pub struct NextFrameSetupSystem<P, R, I, A> {
    m: marker::PhantomData<(P, R, I, A)>,
}

impl<P, R, I, A> NextFrameSetupSystem<P, R, I, A>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    P::Diff: VectorSpace + InnerSpace + Debug,
    R: Rotation<P> + ApplyAngular<P::Scalar, A>,
    I: Inertia<Orientation = R> + Mul<A, Output = A>,
    A: Mul<P::Scalar, Output = A> + Zero + Clone + Copy,
{
    /// Create system.
    pub fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R, I, A> System<'a> for NextFrameSetupSystem<P, R, I, A>
where
    P: EuclideanSpace + Send + Sync + 'static,
    P::Scalar: BaseFloat + Send + Sync + 'static,
    P::Diff: VectorSpace + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + ApplyAngular<P::Scalar, A> + Send + Sync + 'static,
    I: Inertia<Orientation = R> + Mul<A, Output = A> + Send + Sync + 'static,
    A: Mul<P::Scalar, Output = A> + Zero + Clone + Copy + Send + Sync + 'static,
{
    type SystemData = (
        Fetch<'a, DeltaTime<P::Scalar>>,
        ReadStorage<'a, Mass<P::Scalar, I>>,
        WriteStorage<'a, NextFrame<Velocity<P::Diff, A>>>,
        ReadStorage<'a, BodyPose<P, R>>,
        WriteStorage<'a, NextFrame<BodyPose<P, R>>>,
        WriteStorage<'a, ForceAccumulator<P::Diff, A>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, masses, mut next_velocities, poses, mut next_poses, mut forces) = data;

        // Do force integration
        next_frame_integration(
            (&mut next_velocities, &next_poses, &mut forces, &masses).join(),
            time.delta_seconds,
        );

        // Compute next frames position
        next_frame_pose(
            (&next_velocities, &poses, &mut next_poses).join(),
            time.delta_seconds,
        );
    }
}
