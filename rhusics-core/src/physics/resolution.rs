use std::fmt::Debug;
use std::marker;
use std::ops::{Add, Mul, Sub};

use cgmath::{BaseFloat, EuclideanSpace, InnerSpace, One, Rotation, Zero};
use cgmath::num_traits::{Float, NumCast};
use collision::Contact;

use super::{Inertia, Mass, Material, PartialCrossProduct, Velocity};
use {NextFrame, Pose};

const POSITIONAL_CORRECTION_PERCENT: f32 = 0.2;
const POSITIONAL_CORRECTION_K_SLOP: f32 = 0.01;

/// Changes computed from contact resolution.
///
/// Optionally contains the new pose and/or velocity of a body after contact resolution.
///
/// ### Type parameters:
///
/// - `B`: Transform type (`BodyPose3` or similar)
/// - `P`: Point type, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
#[derive(Debug, PartialEq)]
pub struct SingleChangeSet<B, P, R, A>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
    A: Clone,
    B: Pose<P, R>,
{
    pose: Option<B>,
    velocity: Option<NextFrame<Velocity<P::Diff, A>>>,
    m: marker::PhantomData<(P, R)>,
}

impl<B, P, R, A> Default for SingleChangeSet<B, P, R, A>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
    A: Clone,
    B: Pose<P, R>,
{
    fn default() -> Self {
        Self {
            pose: None,
            velocity: None,
            m: marker::PhantomData,
        }
    }
}

impl<B, P, R, A> SingleChangeSet<B, P, R, A>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
    A: Clone,
    B: Pose<P, R>,
{
    #[allow(dead_code)]
    fn new(pose: Option<B>, velocity: Option<NextFrame<Velocity<P::Diff, A>>>) -> Self {
        SingleChangeSet {
            pose,
            velocity,
            m: marker::PhantomData,
        }
    }

    fn add_pose(&mut self, pose: Option<B>) {
        self.pose = pose;
    }

    fn add_velocity(&mut self, velocity: Option<NextFrame<Velocity<P::Diff, A>>>) {
        self.velocity = velocity;
    }

    /// Apply any changes to the next frame pose and/or velocity
    pub fn apply(
        self,
        pose: Option<&mut NextFrame<B>>,
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

/// Data used for contact resolution
///
/// ### Type parameters:
///
/// - `B`: Transform type (`BodyPose3` or similar)
/// - `P`: Point type, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
/// - `I`: Inertia, usually `Scalar` or `Matrix3`
pub struct ResolveData<'a, B, P, R, I, A>
where
    P: EuclideanSpace + 'a,
    P::Scalar: BaseFloat,
    R: Rotation<P> + 'a,
    I: 'a,
    A: Clone + 'a,
    B: Pose<P, R> + 'a,
{
    /// Velocity for next frame
    pub velocity: Option<&'a NextFrame<Velocity<P::Diff, A>>>,
    /// Position for next frame
    pub pose: &'a B,
    /// Mass
    pub mass: &'a Mass<P::Scalar, I>,
    /// Material
    pub material: &'a Material,
    m: marker::PhantomData<(P, R)>,
}

impl<'a, B, P, R, I, A> ResolveData<'a, B, P, R, I, A>
where
    P: EuclideanSpace + 'a,
    P::Scalar: BaseFloat,
    R: Rotation<P> + 'a,
    I: 'a,
    A: Clone + 'a,
    B: Pose<P, R> + 'a,
{
    /// Create resolve data
    pub fn new(
        velocity: Option<&'a NextFrame<Velocity<P::Diff, A>>>,
        pose: &'a B,
        mass: &'a Mass<P::Scalar, I>,
        material: &'a Material,
    ) -> Self {
        ResolveData {
            velocity,
            pose,
            mass,
            material,
            m: marker::PhantomData,
        }
    }
}

/// Perform contact resolution.
///
/// Will compute any new poses and/or velocities, by doing impulse resolution of the given contact.
///
/// ### Parameters:
///
/// - `contact`: The contact; contact normal must point from shape A -> B
/// - `a`: Resolution data for shape A
/// - `b`: Resolution data for shape B
///
/// ### Returns
///
/// Tuple of change sets, first change set is for shape A, second change set for shape B.
///
/// ### Type parameters:
///
/// - `B`: Transform type (`BodyPose3` or similar)
/// - `P`: Point type, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
/// - `I`: Inertia, usually `Scalar` or `Matrix3`
/// - `O`: Internal type used for unifying cross products for 2D/3D, usually `Scalar` or `Vector3`
pub fn resolve_contact<'a, B, P, R, I, A, O>(
    contact: &Contact<P>,
    a: &ResolveData<'a, B, P, R, I, A>,
    b: &ResolveData<'a, B, P, R, I, A>,
) -> (SingleChangeSet<B, P, R, A>, SingleChangeSet<B, P, R, A>)
where
    P: EuclideanSpace + 'a,
    P::Scalar: BaseFloat,
    R: Rotation<P> + 'a,
    P::Diff: Debug + Zero + Clone + InnerSpace + PartialCrossProduct<P::Diff, Output = O>,
    O: PartialCrossProduct<P::Diff, Output = P::Diff>,
    A: PartialCrossProduct<P::Diff, Output = P::Diff> + Clone + Zero + 'a,
    &'a A: Sub<O, Output = A> + Add<O, Output = A>,
    I: Inertia<Orientation = R> + Mul<O, Output = O>,
    B: Pose<P, R> + 'a,
{
    let a_velocity = a.velocity.map(|v| v.value.clone()).unwrap_or_default();
    let b_velocity = b.velocity.map(|v| v.value.clone()).unwrap_or_default();
    let a_inverse_mass = a.mass.inverse_mass();
    let b_inverse_mass = b.mass.inverse_mass();
    let total_inverse_mass = a_inverse_mass + b_inverse_mass;

    // Do positional correction, so bodies aren't penetrating as much any longer.
    let (a_position_new, b_position_new) =
        positional_correction(contact, a.pose, b.pose, a_inverse_mass, b_inverse_mass);

    let mut a_set = SingleChangeSet::default();
    a_set.add_pose(a_position_new);
    let mut b_set = SingleChangeSet::default();
    b_set.add_pose(b_position_new);

    // This only happens when we have 2 infinite masses colliding. We only do positional correction
    // for the bodies and return early
    if total_inverse_mass == P::Scalar::zero() {
        return (a_set, b_set);
    }

    let r_a = contact.contact_point - a.pose.transform_point(P::origin());
    let r_b = contact.contact_point - b.pose.transform_point(P::origin());

    let p_a_dot = *a_velocity.linear() + a_velocity.angular().cross(&r_a);
    let p_b_dot = *b_velocity.linear() + b_velocity.angular().cross(&r_b);

    let rv = p_b_dot - p_a_dot;
    let velocity_along_normal = contact.normal.dot(rv);

    // Check if shapes are already separating, if so only do positional correction
    if velocity_along_normal > P::Scalar::zero() {
        return (a_set, b_set);
    }

    // Compute impulse force
    let a_res: P::Scalar = a.material.restitution();
    let b_res: P::Scalar = b.material.restitution();
    let e = a_res.min(b_res);
    let numerator = -(P::Scalar::one() + e) * velocity_along_normal;

    let a_tensor = a.mass.world_inverse_inertia(a.pose.rotation());
    let b_tensor = b.mass.world_inverse_inertia(b.pose.rotation());

    let term3 = contact
        .normal
        .dot((a_tensor * (r_a.cross(&contact.normal))).cross(&r_a));
    let term4 = contact
        .normal
        .dot((b_tensor * (r_b.cross(&contact.normal))).cross(&r_b));

    let j = numerator / (a_inverse_mass + b_inverse_mass + term3 + term4);
    let impulse = contact.normal * j;

    // Compute new velocities based on mass and the computed impulse
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

/// Do positional correction for colliding bodies.
///
/// Will only do correction for a percentage of the penetration depth, to avoid stability issues.
///
/// ### Parameters:
///
/// - `contact`: Contact information, normal must point from A -> B
/// - `a_position`: World coordinates of center of mass for body A
/// - `b_position`: World coordinates of center of mass for body B
/// - `a_inverse_mass`: Inverse mass of body A
/// - `b_inverse_mass`: Inverse mass of body B
///
/// ### Returns:
///
/// Tuple with new positions for the given bodies
///
/// ### Type parameters:
///
/// - `S`: Scalar type
/// - `B`: Transform type (`BodyPose3` or similar)
/// - `P`: Positional quantity, usually `Point2` or `Point3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
fn positional_correction<S, B, P, R>(
    contact: &Contact<P>,
    a_position: &B,
    b_position: &B,
    a_inverse_mass: S,
    b_inverse_mass: S,
) -> (Option<B>, Option<B>)
where
    S: BaseFloat,
    P: EuclideanSpace<Scalar = S>,
    R: Rotation<P>,
    P::Diff: Debug + Zero + Clone + InnerSpace,
    B: Pose<P, R>,
{
    let total_inverse_mass = a_inverse_mass + b_inverse_mass;
    let k_slop: S = NumCast::from(POSITIONAL_CORRECTION_K_SLOP).unwrap();
    let percent: S = NumCast::from(POSITIONAL_CORRECTION_PERCENT).unwrap();
    let correction_penetration_depth = contact.penetration_depth - k_slop;
    let correction_magnitude =
        correction_penetration_depth.max(S::zero()) / total_inverse_mass * percent;
    let correction = contact.normal * correction_magnitude;
    let a_position_new = new_pose(a_position, correction * -a_inverse_mass);
    let b_position_new = new_pose(b_position, correction * b_inverse_mass);
    (Some(a_position_new), Some(b_position_new))
}

fn new_pose<B, P, R>(next_frame: &B, correction: P::Diff) -> B
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
    P::Diff: Clone,
    B: Pose<P, R>,
{
    B::new(*next_frame.position() + correction, *next_frame.rotation())
}

#[cfg(test)]
mod tests {
    use cgmath::{Basis2, One, Point2, Vector2};
    use collision::{CollisionStrategy, Contact};

    use super::*;
    use BodyPose;
    use collide::ContactEvent;

    #[test]
    fn test_resolve_2d_f32() {
        let mass = Mass::<f32, f32>::new_with_inertia(0.5, 0.);
        let material = Material::default();
        let left_velocity = NextFrame {
            value: Velocity::new(Vector2::<f32>::new(1., 0.), 0.),
        };
        let left_pose = BodyPose::new(Point2::origin(), Basis2::one());
        let right_velocity = NextFrame {
            value: Velocity::new(Vector2::new(-2., 0.), 0.),
        };
        let right_pose = BodyPose::new(Point2::new(1., 0.), Basis2::one());
        let contact = ContactEvent::new(
            (1, 2),
            Contact::new_impl(CollisionStrategy::FullResolution, Vector2::new(1., 0.), 0.5),
        );
        let set = resolve_contact(
            &contact.contact,
            &ResolveData::new(Some(&left_velocity), &left_pose, &mass, &material),
            &ResolveData::new(Some(&right_velocity), &right_pose, &mass, &material),
        );
        assert_eq!(
            (
                SingleChangeSet::new(
                    Some(BodyPose::new(
                        Point2::new(-0.04900000075250864, 0.),
                        Basis2::one()
                    )),
                    Some(NextFrame {
                        value: Velocity::new(Vector2::new(-2., 0.), 0.),
                    }),
                ),
                SingleChangeSet::new(
                    Some(BodyPose::new(
                        Point2::new(1.0490000007525087, 0.),
                        Basis2::one()
                    )),
                    Some(NextFrame {
                        value: Velocity::new(Vector2::new(1., 0.), 0.),
                    }),
                )
            ),
            set
        );
    }

    #[test]
    fn test_resolve_2d_f64() {
        let mass = Mass::<f64, f64>::new_with_inertia(0.5, 0.);
        let material = Material::default();
        let left_velocity = NextFrame {
            value: Velocity::new(Vector2::<f64>::new(1., 0.), 0.),
        };
        let left_pose = BodyPose::new(Point2::origin(), Basis2::one());
        let right_velocity = NextFrame {
            value: Velocity::new(Vector2::new(-2., 0.), 0.),
        };
        let right_pose = BodyPose::new(Point2::new(1., 0.), Basis2::one());
        let contact = ContactEvent::new(
            (1, 2),
            Contact::new_impl(CollisionStrategy::FullResolution, Vector2::new(1., 0.), 0.5),
        );
        let set = resolve_contact(
            &contact.contact,
            &ResolveData::new(Some(&left_velocity), &left_pose, &mass, &material),
            &ResolveData::new(Some(&right_velocity), &right_pose, &mass, &material),
        );
        assert_eq!(
            (
                SingleChangeSet::new(
                    Some(BodyPose::new(
                        Point2::new(-0.04900000075250864, 0.),
                        Basis2::one()
                    )),
                    Some(NextFrame {
                        value: Velocity::new(Vector2::new(-2., 0.), 0.),
                    }),
                ),
                SingleChangeSet::new(
                    Some(BodyPose::new(
                        Point2::new(1.0490000007525087, 0.),
                        Basis2::one()
                    )),
                    Some(NextFrame {
                        value: Velocity::new(Vector2::new(1., 0.), 0.),
                    }),
                )
            ),
            set
        );
    }
}
