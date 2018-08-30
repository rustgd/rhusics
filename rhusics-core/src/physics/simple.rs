//! Simple force integration and impulse solver

use std::ops::Mul;

use cgmath::num_traits::Float;
use cgmath::{BaseFloat, EuclideanSpace, InnerSpace, Rotation, VectorSpace, Zero};

use super::{
    ApplyAngular, ForceAccumulator, Inertia, Mass, PhysicalEntity, Velocity, WorldParameters,
};
use {NextFrame, Pose};

/// Do force integration for next frame.
///
/// ### Parameters:
///
/// - `data`: Iterator over tuple with:
///     - Velocity for the next frame, will be updated
///     - Pose for the next frame, used to compute the inertia tensor for the body in the next frame
///     - Force accumulator, will be consumed and added to the velocity
///     - Mass, used by integration
///     - PhysicalEntity, used for gravity and damping calculation
/// - `world_params`: World physics parameters like gravity and global damping
/// - `dt`: Time step
///
/// ### Type parameters:
///
/// - `D`: Iterator type
/// - `P`: Point, usually `Point2` or `Point3`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
/// - `I`: Inertia, usually `Scalar` or `Matrix3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
pub fn next_frame_integration<'a, T, D, P, A, I, R>(
    data: D,
    world_params: &WorldParameters<P::Diff, P::Scalar>,
    dt: P::Scalar,
) where
    D: Iterator<
        Item = (
            &'a mut NextFrame<Velocity<P::Diff, A>>,
            &'a NextFrame<T>,
            &'a mut ForceAccumulator<P::Diff, A>,
            &'a Mass<P::Scalar, I>,
            &'a PhysicalEntity<P::Scalar>,
        ),
    >,
    T: Pose<P, R> + 'a,
    P: EuclideanSpace + 'a,
    P::Scalar: BaseFloat,
    P::Diff: VectorSpace + InnerSpace + 'a,
    I: Inertia<Orientation = R> + Mul<A, Output = A> + 'a,
    A: Mul<P::Scalar, Output = A> + Clone + Copy + Zero + 'a,
    R: Rotation<P> + ApplyAngular<P::Scalar, A> + 'a,
{
    // Do force integration
    for (next_velocity, next_pose, force, mass, entity) in data.filter(|(_, _, _, _, e)| e.active())
    {
        let a = force.consume_force() * mass.inverse_mass()
            + world_params.gravity() * entity.gravity_scale();
        let new_velocity = *next_velocity.value.linear() + a * dt;
        let damp = world_params.entity_damping(entity.damping()).powf(dt);
        next_velocity.value.set_linear(new_velocity * damp);
        let a = mass.world_inverse_inertia(&next_pose.value.rotation()) * force.consume_torque();
        let new_velocity = *next_velocity.value.angular() + a * dt;
        next_velocity.value.set_angular(new_velocity);
    }
}

/// Compute next frame pose
///
/// ### Parameters:
///
/// - `data`: Iterator over tuple with:
///     - Velocity for the next frame, used to compute next frame pose
///     - Pose for the current frame, will be updated
///     - Pose for the next frame, will be updated
/// - `dt`: Time step
///
/// ### Type parameters:
///
/// - `D`: Iterator type
/// - `P`: Point, usually `Point2` or `Point3`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
/// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
pub fn next_frame_pose<'a, B, D, P, A, R>(data: D, dt: P::Scalar)
where
    D: Iterator<
        Item = (
            &'a NextFrame<Velocity<P::Diff, A>>,
            &'a B,
            &'a mut NextFrame<B>,
            &'a PhysicalEntity<P::Scalar>,
        ),
    >,
    B: Pose<P, R> + 'a,
    P: EuclideanSpace + 'a,
    P::Scalar: BaseFloat,
    P::Diff: VectorSpace + InnerSpace + 'a,
    A: Mul<P::Scalar, Output = A> + Clone + Copy + Zero + 'a,
    R: Rotation<P> + ApplyAngular<P::Scalar, A> + 'a,
{
    for (next_velocity, pose, next_pose, _) in data.filter(|(_, _, _, e)| e.active()) {
        next_pose.value = next_velocity.value.apply(pose, dt)
    }
}
