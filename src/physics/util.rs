use cgmath::{Vector2, Vector3};

use Real;

/// Cross product abstraction
pub trait PartialCrossProduct<RHS = Self> {
    /// Output
    type Output;
    /// Compute cross product
    fn cross(&self, other: &RHS) -> Self::Output;
}

impl PartialCrossProduct<Vector2<Real>> for Real {
    type Output = Vector2<Real>;

    fn cross(&self, other: &Vector2<Real>) -> Self::Output {
        Vector2::new(-*self * other.y, *self * other.x)
    }
}

impl PartialCrossProduct for Vector2<Real> {
    type Output = Real;
    fn cross(&self, other: &Vector2<Real>) -> Real {
        self.x * other.y - self.y * other.x
    }
}

impl PartialCrossProduct for Vector3<Real> {
    type Output = Vector3<Real>;
    fn cross(&self, other: &Vector3<Real>) -> Vector3<Real> {
        Vector3::cross(*self, *other)
    }
}
