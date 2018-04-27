extern crate cgmath;
extern crate collision;

use cgmath::{BaseFloat, Decomposed, EuclideanSpace, One, Rotation, Transform};
pub use collision::{Interpolate, TranslationInterpolate};

/// Pose abstraction
pub trait Pose<P, R>: Transform<P>
where
    P: EuclideanSpace,
{
    /// New pose
    fn new(position: P, rotation: R) -> Self;
    /// Set rotation
    fn set_rotation(&mut self, rotation: R);
    /// Set position
    fn set_position(&mut self, position: P);
    /// Read rotation
    fn rotation(&self) -> R;
    /// Read position
    fn position(&self) -> P;
}

pub trait PhysicsTime<S> {
    fn delta_seconds(&self) -> S;
}

impl<P, R> Pose<P, R> for Decomposed<P::Diff, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
{
    fn new(position: P, rotation: R) -> Self {
        Decomposed {
            rot: rotation,
            disp: position.to_vec(),
            scale: P::Scalar::one(),
        }
    }

    fn set_rotation(&mut self, rotation: R) {
        self.rot = rotation;
    }

    fn set_position(&mut self, position: P) {
        self.disp = position.to_vec();
    }

    fn rotation(&self) -> R {
        self.rot
    }

    fn position(&self) -> P {
        P::from_vec(self.disp)
    }
}
