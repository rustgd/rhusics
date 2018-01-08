use cgmath::{BaseFloat, Vector2, Vector3};

/// Cross product abstraction
pub trait PartialCrossProduct<RHS = Self> {
    /// Output
    type Output;
    /// Compute cross product
    fn cross(&self, other: &RHS) -> Self::Output;
}

impl<S> PartialCrossProduct<Vector2<S>> for S
where
    S: BaseFloat,
{
    type Output = Vector2<S>;

    fn cross(&self, other: &Vector2<S>) -> Self::Output {
        Vector2::new(-*self * other.y, *self * other.x)
    }
}

impl<S> PartialCrossProduct for Vector2<S>
where
    S: BaseFloat,
{
    type Output = S;
    fn cross(&self, other: &Vector2<S>) -> S {
        self.x * other.y - self.y * other.x
    }
}

impl<S> PartialCrossProduct for Vector3<S>
where
    S: BaseFloat,
{
    type Output = Vector3<S>;
    fn cross(&self, other: &Vector3<S>) -> Vector3<S> {
        Vector3::cross(*self, *other)
    }
}
