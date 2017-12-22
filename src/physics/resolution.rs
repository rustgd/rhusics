use std::fmt::Debug;
use std::ops::{Add, Mul, Sub};

use cgmath::{EuclideanSpace, InnerSpace, Rotation, Transform, Vector2, Vector3, VectorSpace, Zero};
use cgmath::num_traits::NumCast;

use super::{Inertia, Mass, Material, Velocity};
use {BodyPose, NextFrame, Real};
use collide::ContactEvent;

const POSITIONAL_CORRECTION_PERCENT: f32 = 0.2;
const POSITIONAL_CORRECTION_K_SLOP: f32 = 0.01;

pub struct SingleChangeSet<P, R, A>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
    A: Clone,
{
    pose: Option<BodyPose<P, R>>,
    velocity: Option<NextFrame<Velocity<P::Diff, A>>>,
}

impl<P, R, A> Default for SingleChangeSet<P, R, A>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
    A: Clone,
{
    fn default() -> Self {
        Self {
            pose: None,
            velocity: None,
        }
    }
}

impl<P, R, A> SingleChangeSet<P, R, A>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
    A: Clone,
{
    fn add_pose(&mut self, pose: Option<BodyPose<P, R>>) {
        self.pose = pose;
    }

    fn add_velocity(&mut self, velocity: Option<NextFrame<Velocity<P::Diff, A>>>) {
        self.velocity = velocity;
    }

    pub fn apply(
        self,
        pose: Option<&mut NextFrame<BodyPose<P, R>>>,
        velocity: Option<&mut NextFrame<Velocity<P::Diff, A>>>,
    ) {
        if let (Some(pose), Some(update_pose)) = (pose, self.pose) {
            pose.value = update_pose;
        }
        if let (Some(velocity), Some(update_velocity)) = (velocity, self.velocity) {
            *velocity = update_velocity;
        }
    }
}

/// Data used for linear contact resolution
pub struct ResolveData<'a, P, R, I, A>
where
    P: EuclideanSpace<Scalar = Real> + 'a,
    R: Rotation<P> + 'a,
    I: 'a,
    A: Clone + 'a,
{
    /// Velocity for next frame
    pub velocity: Option<&'a NextFrame<Velocity<P::Diff, A>>>,
    /// Position for next frame
    pub pose: &'a BodyPose<P, R>,
    /// Mass
    pub mass: &'a Mass<I>,
    /// Material
    pub material: &'a Material,
}

/// Linear and angular contact resolution
pub fn resolve_contact<'a, ID, P, R, I, A, O>(
    contact: &ContactEvent<ID, P>,
    a: ResolveData<'a, P, R, I, A>,
    b: ResolveData<'a, P, R, I, A>,
) -> (SingleChangeSet<P, R, A>, SingleChangeSet<P, R, A>)
where
    P: EuclideanSpace<Scalar = Real> + 'a,
    R: Rotation<P> + 'a,
    P::Diff: Debug + Zero + Clone + InnerSpace + Cross<P::Diff, Output = O>,
    O: Cross<P::Diff, Output = P::Diff>,
    A: Cross<P::Diff, Output = P::Diff> + Clone + Zero + 'a,
    &'a A: Sub<O, Output = A> + Add<O, Output = A>,
    I: Inertia<Orientation = R> + From<R> + Mul<O, Output = O>,
{
    let a_velocity = a.velocity
        .map(|v| v.value.clone())
        .unwrap_or(Velocity::default());
    let b_velocity = b.velocity
        .map(|v| v.value.clone())
        .unwrap_or(Velocity::default());
    let a_inverse_mass = a.mass.inverse_mass();
    let b_inverse_mass = b.mass.inverse_mass();
    let total_inverse_mass = a_inverse_mass + b_inverse_mass;

    let (a_position_new, b_position_new) =
        positional_correction(contact, a.pose, b.pose, a_inverse_mass, b_inverse_mass);

    let mut a_set = SingleChangeSet::default();
    a_set.add_pose(a_position_new);
    let mut b_set = SingleChangeSet::default();
    b_set.add_pose(b_position_new);

    // This only happens when we have 2 infinite masses colliding. We only do positional correction
    // for the bodies and return early
    if total_inverse_mass == 0. {
        return (a_set, b_set);
    }

    let r_a = contact.contact.contact_point - a.pose.transform_point(P::origin());
    let r_b = contact.contact.contact_point - b.pose.transform_point(P::origin());

    let p_a_dot = *a_velocity.linear() + a_velocity.angular().cross(&r_a);
    let p_b_dot = *b_velocity.linear() + b_velocity.angular().cross(&r_b);

    let rv = p_a_dot - p_b_dot;
    let velocity_along_normal = contact.contact.normal.dot(rv);

    if velocity_along_normal > 0. {
        return (a_set, b_set);
    }

    let a_res = a.material.restitution();
    let b_res = b.material.restitution();
    let e = a_res.min(b_res);
    let numerator = -(1. + e) * velocity_along_normal;

    let a_tensor = a.mass.world_inverse_inertia(a.pose.rotation());
    let b_tensor = b.mass.world_inverse_inertia(b.pose.rotation());

    let term3 = contact
        .contact
        .normal
        .dot((a_tensor * (r_a.cross(&contact.contact.normal))).cross(&r_a));
    let term4 = contact
        .contact
        .normal
        .dot((b_tensor * (r_b.cross(&contact.contact.normal))).cross(&r_b));

    let j = numerator / (a_inverse_mass + b_inverse_mass + term3 + term4);
    let impulse = contact.contact.normal * j;

    let a_velocity_new = a.velocity.map(|v| NextFrame {
        value: Velocity::new(
            *v.value.linear() - impulse * a_inverse_mass,
            v.value.angular() - a_tensor * r_a.cross(&impulse),
        ),
    });

    let b_velocity_new = b.velocity.map(|v| NextFrame {
        value: Velocity::new(
            *v.value.linear() + impulse * b_inverse_mass,
            v.value.angular() + b_tensor * r_b.cross(&impulse),
        ),
    });

    a_set.add_velocity(a_velocity_new);
    b_set.add_velocity(b_velocity_new);

    (a_set, b_set)
}

/// Cross product abstraction
pub trait Cross<RHS = Self> {
    /// Output
    type Output;
    /// Compute cross product
    fn cross(&self, other: &RHS) -> Self::Output;
}

impl Cross<Vector2<Real>> for Real {
    type Output = Vector2<Real>;

    fn cross(&self, other: &Vector2<Real>) -> Self::Output {
        Vector2::new(-*self * other.y, *self * other.x)
    }
}

impl Cross for Vector2<Real> {
    type Output = Real;
    fn cross(&self, other: &Vector2<Real>) -> Real {
        self.x * other.y - self.y * other.x
    }
}

impl Cross for Vector3<Real> {
    type Output = Vector3<Real>;
    fn cross(&self, other: &Vector3<Real>) -> Vector3<Real> {
        Vector3::cross(*self, *other)
    }
}

/// Linear contact resolution
pub fn linear_resolve_contact<'a, ID, P, R, I, A>(
    contact: &ContactEvent<ID, P>,
    a: ResolveData<'a, P, R, I, A>,
    b: ResolveData<'a, P, R, I, A>,
) -> (SingleChangeSet<P, R, A>, SingleChangeSet<P, R, A>)
where
    P: EuclideanSpace<Scalar = Real> + 'a,
    R: Rotation<P> + 'a,
    P::Diff: Debug + Zero + Clone + InnerSpace,
    A: Clone + Zero + 'a,
    I: Inertia,
{
    let a_velocity = a.velocity
        .map(|v| v.value.linear.clone())
        .unwrap_or(P::Diff::zero());
    let b_velocity = b.velocity
        .map(|v| v.value.linear.clone())
        .unwrap_or(P::Diff::zero());
    let a_inverse_mass = a.mass.inverse_mass();
    let b_inverse_mass = b.mass.inverse_mass();
    let total_inverse_mass = a_inverse_mass + b_inverse_mass;

    let (a_position_new, b_position_new) =
        positional_correction(contact, a.pose, b.pose, a_inverse_mass, b_inverse_mass);

    let mut a_set = SingleChangeSet::default();
    a_set.add_pose(a_position_new);
    let mut b_set = SingleChangeSet::default();
    b_set.add_pose(b_position_new);

    // This only happens when we have 2 infinite masses colliding. We only do positional correction
    // for the bodies and return early
    if total_inverse_mass == 0. {
        return (a_set, b_set);
    }

    let rv = b_velocity - a_velocity;
    let velocity_along_normal = rv.dot(contact.contact.normal);
    // Bodies are already separating, don't do impulse resolution
    if velocity_along_normal > 0. {
        return (a_set, b_set);
    }
    let a_res = a.material.restitution();
    let b_res = b.material.restitution();
    let e = a_res.min(b_res);
    let j = -(1. + e) * velocity_along_normal / total_inverse_mass;

    let impulse = contact.contact.normal * j;
    let a_velocity_new = a.velocity
        .map(|v| new_linear_velocity(v, impulse * -a_inverse_mass));
    let b_velocity_new = b.velocity
        .map(|v| new_linear_velocity(v, impulse * b_inverse_mass));
    a_set.add_velocity(a_velocity_new);
    b_set.add_velocity(b_velocity_new);

    (a_set, b_set)
}

fn positional_correction<ID, P, R>(
    contact: &ContactEvent<ID, P>,
    a_position: &BodyPose<P, R>,
    b_position: &BodyPose<P, R>,
    a_inverse_mass: Real,
    b_inverse_mass: Real,
) -> (Option<BodyPose<P, R>>, Option<BodyPose<P, R>>)
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
    P::Diff: Debug + Zero + Clone + InnerSpace,
{
    let total_inverse_mass = a_inverse_mass + b_inverse_mass;
    let k_slop: Real = NumCast::from(POSITIONAL_CORRECTION_K_SLOP).unwrap();
    let percent: Real = NumCast::from(POSITIONAL_CORRECTION_PERCENT).unwrap();
    let correction_penetration_depth = contact.contact.penetration_depth - k_slop;
    let correction_magnitude = correction_penetration_depth.max(0.) / total_inverse_mass * percent;
    let correction = contact.contact.normal * correction_magnitude;
    let a_position_new = new_pose(a_position, correction * -a_inverse_mass);
    let b_position_new = new_pose(b_position, correction * b_inverse_mass);
    (Some(a_position_new), Some(b_position_new))
}

fn new_pose<P, R>(next_frame: &BodyPose<P, R>, correction: P::Diff) -> BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
    P::Diff: Clone,
{
    BodyPose::new(
        *next_frame.position() + correction,
        next_frame.rotation().clone(),
    )
}

fn new_linear_velocity<L, A>(
    velocity: &NextFrame<Velocity<L, A>>,
    impulse: L,
) -> NextFrame<Velocity<L, A>>
where
    L: VectorSpace<Scalar = Real>,
    A: Clone + Zero,
{
    NextFrame {
        value: Velocity::from_linear(velocity.value.linear + impulse),
    }
}
