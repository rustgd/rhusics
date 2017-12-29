use std::fmt::Debug;
use std::marker;
use std::ops::Mul;

use cgmath::{EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};
use specs::{Fetch, Join, ReadStorage, System, WriteStorage};

use {BodyPose, NextFrame, Real};
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
pub struct NextFrameSetupSystem<P, R, I, A> {
    m: marker::PhantomData<(P, R, I, A)>,
}

impl<P, R, I, A> NextFrameSetupSystem<P, R, I, A>
where
    P: EuclideanSpace<Scalar = Real>,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug,
    R: Rotation<P> + ApplyAngular<A>,
    I: Inertia<Orientation = R> + Mul<A, Output = A>,
    A: Mul<Real, Output = A> + Zero + Clone + Copy,
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
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + ApplyAngular<A> + Send + Sync + 'static,
    I: Inertia<Orientation = R> + Mul<A, Output = A> + Send + Sync + 'static,
    A: Mul<Real, Output = A> + Zero + Clone + Copy + Send + Sync + 'static,
{
    type SystemData = (
        Fetch<'a, DeltaTime>,
        ReadStorage<'a, Mass<I>>,
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
