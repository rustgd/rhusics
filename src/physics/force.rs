use cgmath::{EuclideanSpace, Rotation, Transform, VectorSpace, Zero};

use super::PartialCrossProduct;
use {BodyPose, Real};

/// Force accumulator for a rigid body.
///
/// Will be consumed when doing force integration for the next frame.
///
/// ### Type parameters:
///
/// - `F`: Force type, usually `Vector2` or `Vector3`
/// - `T`: Torque force, usually `Scalar` or `Vector3`
pub struct ForceAccumulator<F, T> {
    force: F,
    torque: T,
}

impl<F, T> ForceAccumulator<F, T>
where
    F: VectorSpace<Scalar = Real> + Zero,
    T: Zero + Copy + Clone,
{
    /// Create a new force accumulator
    pub fn new() -> Self {
        Self {
            force: F::zero(),
            torque: T::zero(),
        }
    }

    /// Add a force vector to the accumulator
    pub fn add_force(&mut self, force: F) {
        self.force = self.force + force;
    }

    /// Add a torque vector to the accumulator
    pub fn add_torque(&mut self, torque: T) {
        self.torque = self.torque + torque;
    }

    /// Add a force on a given point on the body
    ///
    /// If the force vector does not pass directly through the origin of the body, as expressed by
    /// the pose, torque will occur.
    /// Note that no validation is made on the given position to make sure it's actually contained
    /// in the shape of the body.
    ///
    /// ### Parameters:
    ///
    /// - `force`: Force to apply
    /// - `position`: Position on the body to apply the force at.
    /// - `pose`: Current pose of the body, used to compute the world coordinates of the body center
    ///           of mass
    pub fn add_force_at_point<P, R>(&mut self, force: F, position: P, pose: &BodyPose<P, R>)
    where
        P: EuclideanSpace<Scalar = Real, Diff = F>,
        R: Rotation<P>,
        F: PartialCrossProduct<F, Output = T>,
    {
        let current_pos = pose.transform_point(P::origin());
        let r = position - current_pos;
        self.add_force(force);
        self.add_torque(r.cross(&force));
    }

    /// Consume the accumulated force
    ///
    /// Returns he current accumulated force. The force in the accumulator is reset.
    pub fn consume_force(&mut self) -> F {
        let v = self.force.clone();
        self.force = F::zero();
        v
    }

    /// Consume the torque
    ///
    /// Returns the current accumulated torque. The torque in the accumulator is reset.
    pub fn consume_torque(&mut self) -> T {
        let v = self.torque.clone();
        self.torque = T::zero();
        v
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Point2, Point3, Transform, Vector2, Vector3, Zero};

    use super::ForceAccumulator;
    use Real;
    use physics::prelude2d::BodyPose2;
    use physics::prelude3d::BodyPose3;

    #[test]
    fn test_add_force() {
        let mut forces = ForceAccumulator::<Vector2<Real>, Real>::new();
        forces.add_force(Vector2::new(0., 2.));
        forces.add_force(Vector2::new(1.4, 2.));
        assert_eq!(Vector2::new(1.4, 4.), forces.consume_force());
        assert_eq!(Vector2::zero(), forces.consume_force());
        assert_eq!(0., forces.consume_torque());

        let mut forces = ForceAccumulator::<Vector3<Real>, Real>::new();
        forces.add_force(Vector3::new(0., 2., -1.));
        forces.add_force(Vector3::new(1.4, 2., -1.));
        assert_eq!(Vector3::new(1.4, 4., -2.), forces.consume_force());
        assert_eq!(Vector3::zero(), forces.consume_force());
        assert_eq!(0., forces.consume_torque());
    }

    #[test]
    fn test_add_torque() {
        let mut forces = ForceAccumulator::<Vector2<Real>, Real>::new();
        forces.add_torque(0.2);
        forces.add_torque(1.4);
        assert_ulps_eq!(1.6, forces.consume_torque());
        assert_eq!(Vector2::zero(), forces.consume_force());
        assert_eq!(0., forces.consume_torque());

        let mut forces = ForceAccumulator::<Vector3<Real>, Real>::new();
        forces.add_torque(0.2);
        forces.add_torque(1.4);
        assert_ulps_eq!(1.6, forces.consume_torque());
        assert_eq!(Vector3::zero(), forces.consume_force());
        assert_eq!(0., forces.consume_torque());
    }

    #[test]
    fn test_add_force_at_point_2d() {
        let mut forces = ForceAccumulator::<Vector2<Real>, Real>::new();
        // add at origin -> no torque
        forces.add_force_at_point(Vector2::new(1., 1.), Point2::new(0., 0.), &BodyPose2::one());
        assert_eq!(Vector2::new(1., 1.), forces.consume_force());
        assert_eq!(0., forces.consume_torque());
        // add pointed at origin -> no torque
        forces.add_force_at_point(
            Vector2::new(1., 1.),
            Point2::new(-1., -1.),
            &BodyPose2::one(),
        );
        assert_eq!(Vector2::new(1., 1.), forces.consume_force());
        assert_eq!(0., forces.consume_torque());
        // add outside with offset -> torque
        forces.add_force_at_point(
            Vector2::new(1., 1.),
            Point2::new(-1., 0.),
            &BodyPose2::one(),
        );
        assert_eq!(Vector2::new(1., 1.), forces.consume_force());
        assert_eq!(-1., forces.consume_torque());
    }

    #[test]
    fn test_add_force_at_point_3d() {
        let mut forces = ForceAccumulator::<Vector3<Real>, Vector3<Real>>::new();
        // add at origin -> no torque
        forces.add_force_at_point(
            Vector3::new(1., 1., 1.),
            Point3::new(0., 0., 0.),
            &BodyPose3::one(),
        );
        assert_eq!(Vector3::new(1., 1., 1.), forces.consume_force());
        assert_eq!(Vector3::zero(), forces.consume_torque());
        // add pointed at origin -> no torque
        forces.add_force_at_point(
            Vector3::new(1., 1., 1.),
            Point3::new(-1., -1., -1.),
            &BodyPose3::one(),
        );
        assert_eq!(Vector3::new(1., 1., 1.), forces.consume_force());
        assert_eq!(Vector3::zero(), forces.consume_torque());
        // add outside with offset -> torque
        forces.add_force_at_point(
            Vector3::new(1., 1., 1.),
            Point3::new(-1., 0., 0.),
            &BodyPose3::one(),
        );
        assert_eq!(Vector3::new(1., 1., 1.), forces.consume_force());
        assert_eq!(Vector3::new(0., 1., -1.), forces.consume_torque());
    }
}
