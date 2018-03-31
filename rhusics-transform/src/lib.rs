extern crate cgmath;
extern crate collision;

use cgmath::{EuclideanSpace, Transform};
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
    fn rotation(&self) -> &R;
    /// Read position
    fn position(&self) -> &P;
}
