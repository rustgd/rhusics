

/// Trait bound used for transforms throughout the library
pub trait Pose<P>: Transform<P>
    where
        P: EuclideanSpace<Scalar = Real>,
{
    /// The rotational data type used by the concrete implementation
    type Rotation: Rotation<P>;

    /// Borrows the position attribute
    fn position(&self) -> &P;

    /// Borrows the rotation attribute
    fn rotation(&self) -> &Self::Rotation;

    /// Checks to see if the transform is dirty. Used by the collision system to see if bounds need
    /// to be recalculated.
    fn dirty(&self) -> bool;

    /// Borrows the inverse rotation attribute
    fn inverse_rotation(&self) -> &Self::Rotation;
}

