use std::fmt::Debug;

use cgmath::{EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};
use cgmath::num_traits::NumCast;

use super::{Mass, Velocity};
use {BodyPose, NextFrame, Real};
use collide::ContactEvent;

const POSITIONAL_CORRECTION_PERCENT: f32 = 0.2;
const POSITIONAL_CORRECTION_K_SLOP: f32 = 0.01;

/// Data used for linear contact resolution
pub struct LinearResolveData<'a, P, R>
where
    P: EuclideanSpace<Scalar = Real> + 'a,
    R: Rotation<P> + 'a,
{
    /// Velocity for next frame
    pub velocity: Option<&'a NextFrame<Velocity<P::Diff>>>,
    /// Position for next frame
    pub position: Option<&'a NextFrame<BodyPose<P, R>>>,
    /// Mass
    pub mass: Option<&'a Mass>,
}

/// Linear contact resolution
pub fn linear_resolve_contact<'a, ID, P, R>(
    contact: &ContactEvent<ID, P>,
    a: LinearResolveData<'a, P, R>,
    b: LinearResolveData<'a, P, R>,
) -> (
    Option<NextFrame<BodyPose<P, R>>>,
    Option<NextFrame<BodyPose<P, R>>>,
    Option<NextFrame<Velocity<P::Diff>>>,
    Option<NextFrame<Velocity<P::Diff>>>,
)
where
    P: EuclideanSpace<Scalar = Real> + 'a,
    R: Rotation<P> + 'a,
    P::Diff: Debug + Zero + Clone + InnerSpace,
{
    let a_velocity = a.velocity
        .map(|v| v.value.linear.clone())
        .unwrap_or(P::Diff::zero());
    let b_velocity = b.velocity
        .map(|v| v.value.linear.clone())
        .unwrap_or(P::Diff::zero());
    let a_inverse_mass = a.mass.map(|m| m.inverse_mass).unwrap_or(0.);
    let b_inverse_mass = b.mass.map(|m| m.inverse_mass).unwrap_or(0.);
    let total_inverse_mass = a_inverse_mass + b_inverse_mass;
    // This only happens when we have 2 infinite masses colliding. Such a collision is undefined
    if total_inverse_mass == 0. {
        return (None, None, None, None);
    }

    let k_slop : Real = NumCast::from(POSITIONAL_CORRECTION_K_SLOP).unwrap();
    let percent : Real = NumCast::from(POSITIONAL_CORRECTION_PERCENT).unwrap();
    let correction_penetration_depth =
        contact.contact.penetration_depth - k_slop;
    let correction_magnitude =
        correction_penetration_depth.max(0.) / total_inverse_mass * percent;
    let correction = contact.contact.normal * correction_magnitude;
    let a_position_new = a.position
        .map(|p| new_pose(p, correction * -a_inverse_mass));
    let b_position_new = b.position.map(|p| new_pose(p, correction * b_inverse_mass));

    let rv = b_velocity - a_velocity;
    let velocity_along_normal = rv.dot(contact.contact.normal);
    // Bodies are already separating, don't to impulse resolution
    if velocity_along_normal > 0. {
        return (a_position_new, b_position_new, None, None);
    }
    let e = 1.0; // TODO: restitution
    let j = -(1. + e) * velocity_along_normal / total_inverse_mass;

    let impulse = contact.contact.normal * j;
    let a_velocity_new = a.velocity
        .map(|v| new_velocity(v, impulse * -a_inverse_mass));
    let b_velocity_new = b.velocity
        .map(|v| new_velocity(v, impulse * b_inverse_mass));
    (
        a_position_new,
        b_position_new,
        a_velocity_new,
        b_velocity_new,
    )
}

fn new_pose<P, R>(
    next_frame: &NextFrame<BodyPose<P, R>>,
    correction: P::Diff,
) -> NextFrame<BodyPose<P, R>>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
    P::Diff: Clone,
{
    let new_position = *next_frame.value.position() + correction;
    NextFrame {
        value: BodyPose::new(new_position, next_frame.value.rotation().clone()),
    }
}

fn new_velocity<V>(velocity: &NextFrame<Velocity<V>>, impulse: V) -> NextFrame<Velocity<V>>
where
    V: VectorSpace<Scalar = Real>,
{
    NextFrame {
        value: Velocity {
            linear: velocity.value.linear + impulse,
        },
    }
}
