use std::fmt::Debug;
use std::marker;

use cgmath::{BaseFloat, EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};
use core::{NextFrame, Pose, Velocity};
use specs::{Component, Join, ReadStorage, System, WriteStorage};

/// Current frame update system.
///
/// Will update positions and velocities for the current frame, based on `NextFrame` values.
///
/// ### Type parameters:
///
/// - `P`: Positional quantity, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
///
/// ### System function:
///
/// `fn(NextFrame<Velocity>, NextFrame<T>) -> (Velocity, T)`
pub struct CurrentFrameUpdateSystem<P, R, A, T> {
    m: marker::PhantomData<(P, R, A, T)>,
}

impl<P, R, A, T> CurrentFrameUpdateSystem<P, R, A, T>
where
    P: EuclideanSpace,
    P::Diff: VectorSpace + InnerSpace + Debug,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
    A: Clone + Zero,
    T: Pose<P, R>,
{
    /// Create system.
    pub fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R, A, T> System<'a> for CurrentFrameUpdateSystem<P, R, A, T>
where
    P: EuclideanSpace + Send + Sync + 'static,
    P::Diff: VectorSpace + InnerSpace + Debug + Send + Sync + 'static,
    P::Scalar: BaseFloat + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
    A: Clone + Zero + Send + Sync + 'static,
    T: Pose<P, R> + Component + Clone + Send + Sync + 'static,
{
    type SystemData = (
        WriteStorage<'a, Velocity<P::Diff, A>>,
        ReadStorage<'a, NextFrame<Velocity<P::Diff, A>>>,
        WriteStorage<'a, T>,
        ReadStorage<'a, NextFrame<T>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut velocities, next_velocities, mut poses, next_poses) = data;

        // Update current pose
        for (next, pose) in (&next_poses, &mut poses).join() {
            *pose = next.value.clone();
        }

        // Update current velocity
        for (next, velocity) in (&next_velocities, &mut velocities).join() {
            *velocity = next.value.clone();
        }
    }
}
