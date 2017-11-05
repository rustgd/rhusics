use std::fmt::Debug;
use std::marker;

use cgmath::{EuclideanSpace, InnerSpace, Rotation, VectorSpace};
use shrev::{EventChannel, ReaderId};
use specs::{Entity, Fetch, Join, ReadStorage, System, WriteStorage};

use {BodyPose, NextFrame, Real};
use collide::ContactEvent;
use ecs::physics::resources::DeltaTime;
use physics::{linear_resolve_contact, ForceAccumulator, Inertia, LinearResolveData, Mass,
              RigidBody, Velocity};

/// Linear physics solver system.
///
/// Will do contact resolution, update positions and velocities, do force integration and set up
/// the next frames positions and velocities. Only handles linear quantities, no angular movement.
pub struct LinearSolverSystem<P, R, I> {
    contact_reader: ReaderId<ContactEvent<Entity, P>>,
    m: marker::PhantomData<(P, R, I)>,
}

impl<P, R, I> LinearSolverSystem<P, R, I> {
    /// Create a linear contact solver system.
    pub fn new(contact_reader: ReaderId<ContactEvent<Entity, P>>) -> Self {
        Self {
            contact_reader,
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R, I> System<'a> for LinearSolverSystem<P, R, I>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'a + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
    I: Inertia + Send + Sync + 'static,
{
    type SystemData = (
        Fetch<'a, DeltaTime>,
        Fetch<'a, EventChannel<ContactEvent<Entity, P>>>,
        ReadStorage<'a, Mass<I>>,
        ReadStorage<'a, RigidBody>,
        WriteStorage<'a, Velocity<P::Diff>>,
        WriteStorage<'a, NextFrame<Velocity<P::Diff>>>,
        WriteStorage<'a, BodyPose<P, R>>,
        WriteStorage<'a, NextFrame<BodyPose<P, R>>>,
        WriteStorage<'a, ForceAccumulator<P::Diff>>,
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

fn compute_next_frame<P, R, I>(
    poses: &WriteStorage<BodyPose<P, R>>,
    next_velocities: &mut WriteStorage<NextFrame<Velocity<P::Diff>>>,
    next_poses: &mut WriteStorage<NextFrame<BodyPose<P, R>>>,
    masses: &ReadStorage<Mass<I>>,
    forces: &mut WriteStorage<ForceAccumulator<P::Diff>>,
    time: &DeltaTime,
) where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
    I: Inertia + Send + Sync + 'static,
{
    // Do force integration
    for (next_velocity, force, mass) in (&mut *next_velocities, forces, masses).join() {
        let a = force.consume() * mass.inverse_mass();
        let new_velocity = *next_velocity.value.linear() + a * time.delta_seconds;
        next_velocity.value.set_linear(new_velocity);
    }

    // Compute next frames position
    for (next_velocity, pose, next_pose) in (next_velocities, poses, next_poses).join() {
        next_pose.value = BodyPose::new(
            *pose.position() + *next_velocity.value.linear() * time.delta_seconds,
            pose.rotation().clone(),
        );
    }
}

fn update_current_frame<P, R>(
    velocities: &mut WriteStorage<Velocity<P::Diff>>,
    poses: &mut WriteStorage<BodyPose<P, R>>,
    next_velocities: &WriteStorage<NextFrame<Velocity<P::Diff>>>,
    next_poses: &WriteStorage<NextFrame<BodyPose<P, R>>>,
) where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
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

fn contact_resolution<P, R, I>(
    contacts: &EventChannel<ContactEvent<Entity, P>>,
    contact_reader: &mut ReaderId,
    masses: &ReadStorage<Mass<I>>,
    bodies: &ReadStorage<RigidBody>,
    next_velocities: &mut WriteStorage<NextFrame<Velocity<P::Diff>>>,
    next_poses: &mut WriteStorage<NextFrame<BodyPose<P, R>>>,
) where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
    I: Inertia + Send + Sync + 'static,
{
    match contacts.lossy_read(contact_reader) {
        Ok(data) => for contact in data {
            let (update_pose_0, update_pose_1, update_velocity_0, update_velocity_1) =
                linear_resolve_contact(
                    contact,
                    LinearResolveData {
                        velocity: next_velocities.get(contact.bodies.0),
                        position: next_poses.get(contact.bodies.0),
                        mass: masses.get(contact.bodies.0),
                        material: bodies.get(contact.bodies.0).map(|b| b.material()),
                    },
                    LinearResolveData {
                        velocity: next_velocities.get(contact.bodies.1),
                        position: next_poses.get(contact.bodies.1),
                        mass: masses.get(contact.bodies.1),
                        material: bodies.get(contact.bodies.1).map(|b| b.material()),
                    },
                );
            if let (Some(pose), Some(update_pose)) =
                (next_poses.get_mut(contact.bodies.0), update_pose_0)
            {
                *pose = update_pose;
            }
            if let (Some(pose), Some(update_pose)) =
                (next_poses.get_mut(contact.bodies.1), update_pose_1)
            {
                *pose = update_pose;
            }
            if let (Some(velocity), Some(update_velocity)) =
                (next_velocities.get_mut(contact.bodies.0), update_velocity_0)
            {
                *velocity = update_velocity;
            }
            if let (Some(velocity), Some(update_velocity)) =
                (next_velocities.get_mut(contact.bodies.1), update_velocity_1)
            {
                *velocity = update_velocity;
            }
        },
        Err(err) => println!("Error in contacts read: {:?}", err),
    }
}
