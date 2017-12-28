use std::fmt::Debug;
use std::marker;
use std::ops::{Add, Mul, Sub};

use cgmath::{EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};
use shrev::{EventChannel, ReaderId};
use specs::{Entity, Fetch, Join, ReadStorage, System, WriteStorage};

use {BodyPose, NextFrame, Real};
use collide::ContactEvent;
use ecs::physics::resources::DeltaTime;
use physics::{resolve_contact, ApplyAngular, Cross, ForceAccumulator, Inertia, Mass, ResolveData,
              RigidBody, Velocity};
use physics::simple::*;

/// Impulse physics solver system.
///
/// Will update positions and velocities for the current frame
///
/// ### Type parameters:
///
/// - `P`: Positional quantity, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
pub struct ImpulseSolverSystem<P, R, A> {
    m: marker::PhantomData<(P, R, A)>,
}

impl<P, R, A> ImpulseSolverSystem<P, R, A>
where
    P: EuclideanSpace<Scalar = Real>,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug,
    R: Rotation<P>,
    A: Clone + Zero,
{
    /// Create system.
    pub fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R, A> System<'a> for ImpulseSolverSystem<P, R, A>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
    A: Clone + Zero + Send + Sync + 'static,
{
    type SystemData = (
        WriteStorage<'a, Velocity<P::Diff, A>>,
        ReadStorage<'a, NextFrame<Velocity<P::Diff, A>>>,
        WriteStorage<'a, BodyPose<P, R>>,
        ReadStorage<'a, NextFrame<BodyPose<P, R>>>,
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

/// Do contact resolution
///
/// ### Type parameters:
///
/// - `P`: Positional quantity, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
/// - `I`: Inertia, usually `Scalar` or `Matrix3`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
/// - `O`: Internal type used for abstracting over cross products in 2D/3D,
///        usually `Scalar` or `Vector3`
pub struct ContactResolutionSystem<P, R, I, A, O>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    contact_reader: ReaderId<ContactEvent<Entity, P>>,
    m: marker::PhantomData<(R, I, A, O)>,
}

impl<P, R, I, A, O> ContactResolutionSystem<P, R, I, A, O>
where
    P: EuclideanSpace<Scalar = Real>,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Cross<P::Diff, Output = O>,
    R: Rotation<P> + ApplyAngular<A>,
    O: Cross<P::Diff, Output = P::Diff>,
    A: Cross<P::Diff, Output = P::Diff> + Clone + Zero,
    for<'b> &'b A: Sub<O, Output = A> + Add<O, Output = A>,
    I: Inertia<Orientation = R> + Mul<O, Output = O>,
{
    /// Create system.
    pub fn new(contact_reader: ReaderId<ContactEvent<Entity, P>>) -> Self {
        Self {
            contact_reader,
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R, I, A, O> System<'a> for ContactResolutionSystem<P, R, I, A, O>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real>
        + InnerSpace
        + Debug
        + Send
        + Sync
        + 'static
        + Cross<P::Diff, Output = O>,
    R: Rotation<P> + ApplyAngular<A> + Send + Sync + 'static,
    O: Cross<P::Diff, Output = P::Diff>,
    A: Cross<P::Diff, Output = P::Diff> + Clone + Zero + Send + Sync + 'static,
    for<'b> &'b A: Sub<O, Output = A> + Add<O, Output = A>,
    I: Inertia<Orientation = R> + Mul<O, Output = O> + Send + Sync + 'static,
{
    type SystemData = (
        Fetch<'a, EventChannel<ContactEvent<Entity, P>>>,
        ReadStorage<'a, Mass<I>>,
        ReadStorage<'a, RigidBody>,
        WriteStorage<'a, NextFrame<Velocity<P::Diff, A>>>,
        ReadStorage<'a, BodyPose<P, R>>,
        WriteStorage<'a, NextFrame<BodyPose<P, R>>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (contacts, masses, bodies, mut next_velocities, poses, mut next_poses) = data;

        // Process contacts since last run
        for contact in contacts.read(&mut self.contact_reader) {
            // Resolve contact
            let change_set = resolve_contact(
                &contact.contact,
                ResolveData {
                    velocity: next_velocities.get(contact.bodies.0),
                    pose: next_poses
                        .get(contact.bodies.0)
                        .map(|p| &p.value)
                        .unwrap_or_else(|| poses.get(contact.bodies.0).unwrap()),
                    mass: masses.get(contact.bodies.0).unwrap(),
                    material: bodies.get(contact.bodies.0).map(|b| b.material()).unwrap(),
                },
                ResolveData {
                    velocity: next_velocities.get(contact.bodies.1),
                    pose: next_poses
                        .get(contact.bodies.1)
                        .map(|p| &p.value)
                        .unwrap_or_else(|| poses.get(contact.bodies.1).unwrap()),
                    mass: masses.get(contact.bodies.1).unwrap(),
                    material: bodies.get(contact.bodies.1).map(|b| b.material()).unwrap(),
                },
            );
            // Apply computed change sets
            change_set.0.apply(
                next_poses.get_mut(contact.bodies.0),
                next_velocities.get_mut(contact.bodies.0),
            );
            change_set.1.apply(
                next_poses.get_mut(contact.bodies.1),
                next_velocities.get_mut(contact.bodies.1),
            );
        }
    }
}

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
