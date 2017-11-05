use std::fmt::Debug;
use std::marker;

use cgmath::{EuclideanSpace, InnerSpace, Rotation, VectorSpace};
use shrev::{EventChannel, ReaderId};
use specs::{Entity, Fetch, Join, ReadStorage, System, WriteStorage};

use {BodyPose, NextFrame, Real};
use collide::ContactEvent;
use ecs::physics::resources::DeltaTime;
use physics::{linear_resolve_contact, LinearResolveData, Mass, Velocity};
use physics::prelude2d::Mass2;

/// Linear contact solver system.
///
/// Will do contact resolution, update positions and velocities and set up the next frames positions
/// and velocities.
pub struct LinearContactSolverSystem<P, R> {
    contact_reader: ReaderId,
    m: marker::PhantomData<(P, R)>,
}

impl<P, R> LinearContactSolverSystem<P, R> {
    /// Create a linear contact solver system.
    pub fn new(contact_reader: ReaderId) -> Self {
        Self {
            contact_reader,
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R> System<'a> for LinearContactSolverSystem<P, R>
where
    P: EuclideanSpace<Scalar = Real> + Send + Sync + 'a + 'static,
    P::Diff: VectorSpace<Scalar = Real> + InnerSpace + Debug + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
{
    type SystemData = (
        Fetch<'a, DeltaTime>,
        Fetch<'a, EventChannel<ContactEvent<Entity, P>>>,
        ReadStorage<'a, Mass2>,
        WriteStorage<'a, Velocity<P::Diff>>,
        WriteStorage<'a, NextFrame<Velocity<P::Diff>>>,
        WriteStorage<'a, BodyPose<P, R>>,
        WriteStorage<'a, NextFrame<BodyPose<P, R>>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            contacts,
            masses,
            mut velocities,
            mut next_velocities,
            mut poses,
            mut next_poses,
        ) = data;
        match contacts.lossy_read(&mut self.contact_reader) {
            Ok(data) => for contact in data {
                let (update_pose_0, update_pose_1, update_velocity_0, update_velocity_1) =
                    linear_resolve_contact(
                        contact,
                        LinearResolveData {
                            velocity: next_velocities.get(contact.bodies.0),
                            position: next_poses.get(contact.bodies.0),
                            mass: masses.get(contact.bodies.0),
                        },
                        LinearResolveData {
                            velocity: next_velocities.get(contact.bodies.1),
                            position: next_poses.get(contact.bodies.1),
                            mass: masses.get(contact.bodies.1),
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

        // Update current pose
        for (next, pose) in (&next_poses, &mut poses).join() {
            *pose = next.value.clone();
        }

        // Update current velocity
        for (next, velocity) in (&next_velocities, &mut velocities).join() {
            *velocity = next.value.clone();
        }

        // Compute next frames position + velocity
        for (velocity, next_velocity, pose, next_pose) in
            (&velocities, &mut next_velocities, &poses, &mut next_poses).join()
        {
            next_pose.value = BodyPose::new(
                *pose.position() + *velocity.linear() * time.delta_seconds,
                pose.rotation().clone(),
            );
            next_velocity.value = Velocity::from_linear(*velocity.linear());
        }
    }
}
