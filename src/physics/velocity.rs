use cgmath::{BaseFloat, Basis2, EuclideanSpace, Euler, Quaternion, Rad, Rotation, Rotation2,
             Vector3, VectorSpace, Zero};

use BodyPose;

/// Velocity
///
/// ### Type parameters:
///
/// - `L`: Linear velocity, usually `Vector2` or `Vector3`
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
#[derive(Debug, Clone, PartialEq)]
pub struct Velocity<L, A> {
    linear: L,
    angular: A,
}

impl<L, A> Default for Velocity<L, A>
where
    L: Zero,
    A: Clone + Zero,
{
    fn default() -> Self {
        Self::new(L::zero(), A::zero())
    }
}

impl<L, A> Velocity<L, A>
where
    L: Zero,
    A: Clone + Zero,
{
    /// Create new velocity object, with both linear and angular velocity
    pub fn new(linear: L, angular: A) -> Self {
        Self { linear, angular }
    }

    /// Create new velocity object with only linear velocity
    pub fn from_linear(linear: L) -> Self {
        Self::new(linear, A::zero())
    }

    /// Set linear velocity
    pub fn set_linear(&mut self, linear: L) {
        self.linear = linear;
    }

    /// Get linear velocity
    pub fn linear(&self) -> &L {
        &self.linear
    }

    /// Set angular velocity
    pub fn set_angular(&mut self, angular: A) {
        self.angular = angular;
    }

    /// Get angular velocity
    pub fn angular(&self) -> &A {
        &self.angular
    }

    /// Apply velocity to pose.
    ///
    /// ### Parameters:
    ///
    /// - `pose`: Pose to apply the velocity to
    /// - `dt`: Time step
    ///
    /// ### Type parameters:
    ///
    /// - `P`: Positional quantity, usually `Point2` or `Point3`
    /// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
    pub fn apply<P, R>(&self, pose: &BodyPose<P, R>, dt: L::Scalar) -> BodyPose<P, R>
    where
        P: EuclideanSpace<Scalar = L::Scalar, Diff = L>,
        L: VectorSpace,
        L::Scalar: BaseFloat,
        R: ApplyAngular<L::Scalar, A> + Rotation<P>,
    {
        BodyPose::new(
            self.apply_linear(pose.position(), dt),
            self.apply_angular(pose.rotation(), dt),
        )
    }

    /// Apply linear velocity to a positional quantity
    ///
    /// ### Parameters:
    ///
    /// - `linear`: Positional value
    /// - `dt`: Time step
    ///
    /// ### Type parameters:
    ///
    /// - `P`: Positional quantity, usually `Point2` or `Point3`
    pub fn apply_linear<P>(&self, linear: &P, dt: L::Scalar) -> P
    where
        P: EuclideanSpace<Scalar = L::Scalar, Diff = L>,
        L::Scalar: BaseFloat,
        L: VectorSpace,
    {
        *linear + self.linear * dt
    }

    /// Apply angular velocity to a rotational quantity
    ///
    /// ### Parameters:
    ///
    /// - `rotation`: Rotational value
    /// - `dt`: Time step
    ///
    /// ### Type parameters:
    ///
    /// - `R`: Rotational quantity, usually `Basis2` or `Quaternion`
    pub fn apply_angular<R>(&self, rotation: &R, dt: L::Scalar) -> R
    where
        R: ApplyAngular<L::Scalar, A>,
        L: VectorSpace,
        L::Scalar: BaseFloat,
    {
        rotation.apply(&self.angular, dt)
    }
}

/// Apply an angular velocity to a rotational quantity
///
/// ### Type parameters:
///
/// - `A`: Angular velocity, usually `Scalar` or `Vector3`
pub trait ApplyAngular<S, A> {
    /// Apply given velocity
    fn apply(&self, velocity: &A, dt: S) -> Self;
}

impl<S> ApplyAngular<S, S> for S
where
    S: BaseFloat,
{
    fn apply(&self, velocity: &S, dt: S) -> Self {
        *self + *velocity * dt
    }
}

impl<S> ApplyAngular<S, S> for Basis2<S>
where
    S: BaseFloat,
{
    fn apply(&self, velocity: &S, dt: S) -> Self {
        *self * Basis2::from_angle(Rad(*velocity * dt))
    }
}

impl<S> ApplyAngular<S, Vector3<S>> for Quaternion<S>
where
    S: BaseFloat,
{
    fn apply(&self, velocity: &Vector3<S>, dt: S) -> Self {
        self * Quaternion::from(Euler {
            x: Rad(velocity.x * dt),
            y: Rad(velocity.y * dt),
            z: Rad(velocity.z * dt),
        })
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Basis2, Point2, Point3, Rad, Rotation2, Rotation3, Transform, Vector2};

    use super::*;
    use physics::prelude2d::BodyPose2;
    use physics::prelude3d::BodyPose3;

    #[test]
    fn test_velocity_linear() {
        let velocity = Velocity::new(Vector2::new(1., 1.), 0.);
        let pose = Point2::new(0., 0.);
        let pose = velocity.apply_linear(&pose, 0.1);
        assert_eq!(Point2::new(0.1, 0.1), pose);
    }

    #[test]
    fn test_velocity_2d_angular() {
        let velocity = Velocity::new(Vector2::new(1., 1.), 1.);
        let orientation = Basis2::from_angle(Rad(0.));
        let orientation = velocity.apply_angular(&orientation, 0.1);
        assert_eq!(Basis2::from_angle(Rad(0.1)), orientation);
    }

    #[test]
    fn test_velocity_3d_angular() {
        let velocity = Velocity::new(Vector3::new(1., 1., 1.), Vector3::new(0., 1., 0.));

        let orientation = Quaternion::from_angle_y(Rad(0.));
        let orientation = velocity.apply_angular(&orientation, 0.1);
        assert_eq!(Quaternion::from_angle_y(Rad(0.1)), orientation);
    }

    #[test]
    fn test_velocity_full_2d() {
        let velocity = Velocity::new(Vector2::new(1., 1.), 1.);
        let pose = BodyPose2::one();
        let pose = velocity.apply(&pose, 0.1);
        assert_eq!(Point2::new(0.1, 0.1), *pose.position());
        assert_eq!(Basis2::from_angle(Rad(0.1)), *pose.rotation());
    }

    #[test]
    fn test_velocity_full_3d() {
        let velocity = Velocity::new(Vector3::new(1., 1., 1.), Vector3::new(0., 1., 0.));
        let pose = BodyPose3::one();
        let pose = velocity.apply(&pose, 0.1);
        assert_eq!(Point3::new(0.1, 0.1, 0.1), *pose.position());
        assert_eq!(Quaternion::from_angle_y(Rad(0.1)), *pose.rotation());
    }

    #[test]
    fn test_apply_angular_basis2() {
        let orientation = Basis2::from_angle(Rad(0.));
        let velocity = 0.5;
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);

        assert_ulps_eq!(Basis2::from_angle(Rad(0.2)), orientation);
    }

    #[test]
    fn test_apply_angular_real() {
        let orientation = 0.;
        let velocity = 0.5;
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);

        assert_eq!(0.2, orientation);
    }

    #[test]
    fn test_apply_angular_quat() {
        let orientation = Quaternion::from_angle_x(Rad(0.));
        let velocity = Vector3::new(0.5, 0., 0.);
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);
        let orientation = orientation.apply(&velocity, 0.1);

        assert_ulps_eq!(Quaternion::from_angle_x(Rad(0.2)), orientation);
    }
}
