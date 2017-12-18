use std::fmt::Debug;
use std::marker;
use std::ops::{Add, Mul, Sub};

use cgmath::{EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};
use shrev::{EventChannel, ReaderId};
use specs::{Entity, Fetch, Join, ReadStorage, System, WriteStorage};

use {BodyPose, NextFrame, Real};
use collide::ContactEvent;
use ecs::physics::resources::DeltaTime;
use physics::{linear_resolve_contact, resolve_contact, ApplyAngular, Cross, ForceAccumulator,
              Inertia, Mass, ResolveData, RigidBody, Velocity};

/// Impulse physics solver system.
///
/// Will update positions and velocities for the current frame
pub struct ImpulseSolverSystem<P, R, A> {
    m: marker::PhantomData<(P, R, A)>,
}

impl<P, R, A> ImpulseSolverSystem<P, R, A> {
    /// Create system.
    pub fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R, A> System<'a> for ImpulseSolverSystem<P, R, A>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'a + 'static,
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

/// Does contact resolution
pub struct ContactResolutionSystem<P, R, I, A, O>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    contact_reader: ReaderId<ContactEvent<Entity, P>>,
    m: marker::PhantomData<(R, I, A, O)>,
    linear_only: bool,
}

impl<P, R, I, A, O> ContactResolutionSystem<P, R, I, A, O>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    /// Create system.
    pub fn new(contact_reader: ReaderId<ContactEvent<Entity, P>>) -> Self {
        Self {
            contact_reader,
            m: marker::PhantomData,
            linear_only: false,
        }
    }

    /// Set system to only to linear integration
    pub fn with_linear_only(&mut self) {
        self.linear_only = true;
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
    I: Inertia<Orientation = R> + From<R> + Mul<O, Output = O> + Send + Sync + 'static,
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

        for contact in contacts.read(&mut self.contact_reader) {
            let change_set = {
                let f = if self.linear_only {
                    linear_resolve_contact
                } else {
                    resolve_contact
                };
                f(
                    contact,
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
                )
            };
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
pub struct NextFrameSetupSystem<P, R, I, A> {
    m: marker::PhantomData<(P, R, I, A)>,
    linear_only: bool,
}

impl<P, R, I, A> NextFrameSetupSystem<P, R, I, A> {
    /// Create system.
    pub fn new() -> Self {
        Self {
            m: marker::PhantomData,
            linear_only: false,
        }
    }

    /// Set system to only to linear integration
    pub fn with_linear_only(&mut self) {
        self.linear_only = true;
    }
}

impl<'a, P, R, I, A> System<'a> for NextFrameSetupSystem<P, R, I, A>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + ApplyAngular<A> + Send + Sync + 'static,
    I: Inertia<Orientation = R> + Mul<A, Output = A> + From<R> + Send + Sync + 'static,
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
        for (next_velocity, next_pose, force, mass) in
            (&mut next_velocities, &next_poses, &mut forces, &masses).join()
        {
            let a = force.consume_force() * mass.inverse_mass();
            let new_velocity = *next_velocity.value.linear() + a * time.delta_seconds;
            next_velocity.value.set_linear(new_velocity);
            if !self.linear_only {
                let a =
                    mass.world_inverse_inertia(next_pose.value.rotation()) * force.consume_torque();
                let new_velocity = *next_velocity.value.angular() + a * time.delta_seconds;
                next_velocity.value.set_angular(new_velocity);
            }
        }

        // Compute next frames position
        for (next_velocity, pose, next_pose) in (&next_velocities, &poses, &mut next_poses).join() {
            next_pose.value = next_velocity
                .value
                .apply(pose, time.delta_seconds, self.linear_only)
        }
    }
}
