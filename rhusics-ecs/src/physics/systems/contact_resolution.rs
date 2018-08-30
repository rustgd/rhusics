use std::fmt::Debug;
use std::marker;
use std::ops::{Add, Mul, Sub};

use cgmath::{BaseFloat, EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};
use core::{
    resolve_contact, ApplyAngular, ContactEvent, Inertia, Mass, PartialCrossProduct,
    PhysicalEntity, ResolveData, Velocity,
};
use core::{NextFrame, Pose};
use shrev::{EventChannel, ReaderId};
use specs::prelude::{Component, Entity, Read, ReadStorage, Resources, System, WriteStorage};

/// Do single contact, forward resolution.
///
/// ### Type parameters:
///
/// - `P`: Positional quantity, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
/// - `I`: Inertia, usually `Scalar` or `Matrix3`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
/// - `O`: Internal type used for abstracting over cross products in 2D/3D,
///        usually `Scalar` or `Vector3`
/// - `T`: Transform type (`BodyPose2` or similar)
///
/// ### System function
///
/// `fn(EventChannel<ContactEvent>, Mass, PhysicalEntity, T, NextFrame<Velocity>, NextFrame<T>) -> (NextFrame<Velocity>, NextFrame<T>)`
///
pub struct ContactResolutionSystem<P, R, I, A, O, T>
where
    P: EuclideanSpace + 'static,
    P::Diff: Debug,
{
    contact_reader: Option<ReaderId<ContactEvent<Entity, P>>>,
    m: marker::PhantomData<(R, I, A, O, T)>,
}

impl<P, R, I, A, O, T> ContactResolutionSystem<P, R, I, A, O, T>
where
    T: Pose<P, R>,
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    P::Diff: VectorSpace + InnerSpace + Debug + PartialCrossProduct<P::Diff, Output = O>,
    R: Rotation<P> + ApplyAngular<P::Scalar, A>,
    O: PartialCrossProduct<P::Diff, Output = P::Diff>,
    A: PartialCrossProduct<P::Diff, Output = P::Diff> + Clone + Zero,
    for<'b> &'b A: Sub<O, Output = A> + Add<O, Output = A>,
    I: Inertia<Orientation = R> + Mul<O, Output = O>,
{
    /// Create system.
    pub fn new() -> Self {
        Self {
            contact_reader: None,
            m: marker::PhantomData,
        }
    }
}

impl<'a, P, R, I, A, O, T> System<'a> for ContactResolutionSystem<P, R, I, A, O, T>
where
    T: Pose<P, R> + Component + Send + Sync + 'static,
    P: EuclideanSpace + Send + Sync + 'static,
    P::Scalar: BaseFloat + Send + Sync + 'static,
    P::Diff: VectorSpace
        + InnerSpace
        + Debug
        + Send
        + Sync
        + 'static
        + PartialCrossProduct<P::Diff, Output = O>,
    R: Rotation<P> + ApplyAngular<P::Scalar, A> + Send + Sync + 'static,
    O: PartialCrossProduct<P::Diff, Output = P::Diff>,
    A: PartialCrossProduct<P::Diff, Output = P::Diff> + Clone + Zero + Send + Sync + 'static,
    for<'b> &'b A: Sub<O, Output = A> + Add<O, Output = A>,
    I: Inertia<Orientation = R> + Mul<O, Output = O> + Send + Sync + 'static,
{
    type SystemData = (
        Read<'a, EventChannel<ContactEvent<Entity, P>>>,
        ReadStorage<'a, Mass<P::Scalar, I>>,
        ReadStorage<'a, PhysicalEntity<P::Scalar>>,
        WriteStorage<'a, NextFrame<Velocity<P::Diff, A>>>,
        ReadStorage<'a, T>,
        WriteStorage<'a, NextFrame<T>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (contacts, masses, entities, mut next_velocities, poses, mut next_poses) = data;

        // Process contacts since last run
        for contact in contacts.read(&mut self.contact_reader.as_mut().unwrap()) {
            // Resolve contact
            let change_set = match (
                from_storage(
                    contact.bodies.0,
                    &next_velocities,
                    &next_poses,
                    &poses,
                    &masses,
                    &entities,
                ),
                from_storage(
                    contact.bodies.1,
                    &next_velocities,
                    &next_poses,
                    &poses,
                    &masses,
                    &entities,
                ),
            ) {
                (Some(resolve_0), Some(resolve_1)) => {
                    Some(resolve_contact(&contact.contact, &resolve_0, &resolve_1))
                }
                _ => None,
            };
            if let Some(cs) = change_set {
                // Apply computed change sets
                cs.0.apply(
                    next_poses.get_mut(contact.bodies.0),
                    next_velocities.get_mut(contact.bodies.0),
                );
                cs.1.apply(
                    next_poses.get_mut(contact.bodies.1),
                    next_velocities.get_mut(contact.bodies.1),
                );
            }
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        use specs::prelude::SystemData;
        Self::SystemData::setup(res);
        self.contact_reader = Some(
            res.fetch_mut::<EventChannel<ContactEvent<Entity, P>>>()
                .register_reader(),
        );
    }
}

fn from_storage<'a, P, T, R, A, I>(
    entity: Entity,
    next_velocities: &'a WriteStorage<NextFrame<Velocity<P::Diff, A>>>,
    next_poses: &'a WriteStorage<NextFrame<T>>,
    poses: &'a ReadStorage<T>,
    masses: &'a ReadStorage<Mass<P::Scalar, I>>,
    entities: &'a ReadStorage<PhysicalEntity<P::Scalar>>,
) -> Option<ResolveData<'a, T, P, R, I, A>>
where
    P: EuclideanSpace + Send + Sync + 'static,
    P::Scalar: BaseFloat + Send + Sync + 'static,
    P::Diff: Send + Sync + 'static,
    T: Pose<P, R> + Component + Send + Sync + 'static,
    R: Rotation<P> + Send + Sync + 'static,
    A: Clone + Send + Sync + 'static,
    I: Clone + Send + Sync + 'static,
{
    match (entities.get(entity), masses.get(entity), poses.get(entity)) {
        (Some(e), Some(mass), Some(pose)) if e.active() => Some(ResolveData::new(
            next_velocities.get(entity),
            next_poses.get(entity).map(|p| &p.value).unwrap_or(pose),
            mass,
            e.material(),
        )),
        _ => None,
    }
}
