use std::fmt::Debug;
use std::marker;
use std::ops::{Add, Mul, Sub};

use cgmath::{EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};
use shrev::{EventChannel, ReaderId};
use specs::{Entity, Fetch, ReadStorage, System, WriteStorage};

use {BodyPose, NextFrame, Real};
use collide::ContactEvent;
use physics::{resolve_contact, ApplyAngular, Inertia, Mass, PartialCrossProduct, ResolveData,
              RigidBody, Velocity};

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
///
/// ### System function
///
/// `fn(EventChannel<ContactEvent>, Mass, RigidBody, BodyPose, NextFrame<Velocity>, NextFrame<BodyPose>) -> (NextFrame<Velocity>, NextFrame<BodyPose>)`
///
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
    P::Diff: VectorSpace<Scalar = Real>
        + InnerSpace
        + Debug
        + PartialCrossProduct<P::Diff, Output = O>,
    R: Rotation<P> + ApplyAngular<A>,
    O: PartialCrossProduct<P::Diff, Output = P::Diff>,
    A: PartialCrossProduct<P::Diff, Output = P::Diff> + Clone + Zero,
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
        + PartialCrossProduct<P::Diff, Output = O>,
    R: Rotation<P> + ApplyAngular<A> + Send + Sync + 'static,
    O: PartialCrossProduct<P::Diff, Output = P::Diff>,
    A: PartialCrossProduct<P::Diff, Output = P::Diff> + Clone + Zero + Send + Sync + 'static,
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
