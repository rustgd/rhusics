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

/// Impulse physics solver system.
///
/// Will do contact resolution, update positions and velocities, do force integration and set up
/// the next frames positions and velocities.
pub struct ImpulseSolverSystem<P, R, I, A, O> {
    contact_reader: ReaderId,
    m: marker::PhantomData<(P, R, I, A, O)>,
}

impl<P, R, I, A, O> ImpulseSolverSystem<P, R, I, A, O> {
    /// Create an impulse contact solver system.
    pub fn new(contact_reader: ReaderId) -> Self {
        Self {
            contact_reader,
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R, I, A, O> System<'a> for ImpulseSolverSystem<P, R, I, A, O>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'a + 'static,
    P::Diff: VectorSpace<Scalar = Real>
        + InnerSpace
        + Debug
        + Send
        + Sync
        + 'static
        + Cross<P::Diff, Output = O>
        + Mul<I, Output = P::Diff>,
    R: Rotation<P> + ApplyAngular<A> + Send + Sync + 'static,
    O: Cross<P::Diff, Output = P::Diff>,
    A: Cross<P::Diff, Output = P::Diff>
        + Mul<Real, Output = A>
        + Clone
        + Copy
        + Zero
        + Send
        + Sync
        + 'static,
    for<'b> &'b A: Sub<O, Output = A> + Add<O, Output = A>,
    I: Inertia<Orientation = R>
        + Mul<A, Output = A>
        + From<R>
        + Mul<O, Output = O>
        + Send
        + Sync
        + 'static,
{
    type SystemData = (
        Fetch<'a, DeltaTime>,
        Fetch<'a, EventChannel<ContactEvent<Entity, P>>>,
        ReadStorage<'a, Mass<I>>,
        ReadStorage<'a, RigidBody>,
        WriteStorage<'a, Velocity<P::Diff, A>>,
        WriteStorage<'a, NextFrame<Velocity<P::Diff, A>>>,
        WriteStorage<'a, BodyPose<P, R>>,
        WriteStorage<'a, NextFrame<BodyPose<P, R>>>,
        WriteStorage<'a, ForceAccumulator<P::Diff, A>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            contacts,
            masses,
            bodies,
            mut velocities,
            mut next_velocities,
            mut poses,
            mut next_poses,
            mut forces,
        ) = data;

        contact_resolution(
            &contacts,
            &mut self.contact_reader,
            &masses,
            &bodies,
            &mut next_velocities,
            &mut next_poses,
            &poses,
        );

        update_current_frame(&mut velocities, &mut poses, &next_velocities, &next_poses);

        compute_next_frame(
            &poses,
            &mut next_velocities,
            &mut next_poses,
            &masses,
            &mut forces,
            &*time,
        );
    }
}

fn compute_next_frame<P, R, I, A>(
    poses: &WriteStorage<BodyPose<P, R>>,
    next_velocities: &mut WriteStorage<NextFrame<Velocity<P::Diff, A>>>,
    next_poses: &mut WriteStorage<NextFrame<BodyPose<P, R>>>,
    masses: &ReadStorage<Mass<I>>,
    forces: &mut WriteStorage<ForceAccumulator<P::Diff, A>>,
    time: &DeltaTime,
) where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + ApplyAngular<A> + Send + Sync + 'static,
    I: Inertia<Orientation = R> + Mul<A, Output = A> + From<R> + Send + Sync + 'static,
    A: Mul<Real, Output = A> + Zero + Clone + Copy + Send + Sync + 'static,
{
    // Do force integration
    for (next_velocity, next_pose, force, mass) in
        (&mut *next_velocities, &*next_poses, forces, masses).join()
    {
        let a = force.consume_force() * mass.inverse_mass();
        let new_velocity = *next_velocity.value.linear() + a * time.delta_seconds;
        next_velocity.value.set_linear(new_velocity);
        let a = mass.world_inverse_inertia(next_pose.value.rotation()) * force.consume_torque();
        let new_velocity = *next_velocity.value.angular() + a * time.delta_seconds;
        next_velocity.value.set_angular(new_velocity);
    }

    // Compute next frames position
    for (next_velocity, pose, next_pose) in (next_velocities, poses, next_poses).join() {
        next_pose.value = next_velocity.value.apply(pose, time.delta_seconds)
    }
}

fn update_current_frame<P, R, A>(
    velocities: &mut WriteStorage<Velocity<P::Diff, A>>,
    poses: &mut WriteStorage<BodyPose<P, R>>,
    next_velocities: &WriteStorage<NextFrame<Velocity<P::Diff, A>>>,
    next_poses: &WriteStorage<NextFrame<BodyPose<P, R>>>,
) where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
    A: Clone + Send + Sync + 'static,
{
    // Update current pose
    for (next, pose) in (next_poses, poses).join() {
        *pose = next.value.clone();
    }

    // Update current velocity
    for (next, velocity) in (next_velocities, velocities).join() {
        *velocity = next.value.clone();
    }
}

fn contact_resolution<P, R, I, A, O>(
    contacts: &EventChannel<ContactEvent<Entity, P>>,
    contact_reader: &mut ReaderId,
    masses: &ReadStorage<Mass<I>>,
    bodies: &ReadStorage<RigidBody>,
    next_velocities: &mut WriteStorage<NextFrame<Velocity<P::Diff, A>>>,
    next_poses: &mut WriteStorage<NextFrame<BodyPose<P, R>>>,
    poses: &WriteStorage<BodyPose<P, R>>,
) where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real>
        + InnerSpace
        + Debug
        + Send
        + Sync
        + 'static
        + Cross<P::Diff, Output = O>
        + Mul<I, Output = P::Diff>,
    R: Rotation<P> + Send + Sync + 'static,
    O: Cross<P::Diff, Output = P::Diff>,
    A: Cross<P::Diff, Output = P::Diff> + Clone + Zero + Send + Sync + 'static,
    for<'a> &'a A: Sub<O, Output = A> + Add<O, Output = A>,
    I: Inertia<Orientation = R> + From<R> + Mul<O, Output = O> + Send + Sync + 'static,
{
    match contacts.lossy_read(contact_reader) {
        Ok(data) => for contact in data {
            let change_set = resolve_contact(
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
            );
            change_set.0.apply(
                next_poses.get_mut(contact.bodies.0),
                next_velocities.get_mut(contact.bodies.0),
            );
            change_set.1.apply(
                next_poses.get_mut(contact.bodies.1),
                next_velocities.get_mut(contact.bodies.1),
            );
        },
        Err(err) => println!("Error in contacts read: {:?}", err),
    }
}
