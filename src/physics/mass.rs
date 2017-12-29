use std;
use std::ops::Mul;

use cgmath::{Basis2, Matrix, Matrix3, Quaternion, SquareMatrix, Zero};

use super::{Material, Volume};
use Real;

/// Mass
///
/// Mass properties for a body. Inertia is the moment of inertia in the base pose. For retrieving
/// the inertia tensor for the body in its current orientation, see `world_inertia` and
/// `world_inverse_inertia`.
///
/// ### Type parameters:
///
/// - `I`: Inertia type, usually `Scalar` or `Matrix3`, see `Inertia` for more information.
#[derive(Debug)]
pub struct Mass<I> {
    mass: Real,
    inverse_mass: Real,

    inertia: I,
    inverse_inertia: I,
}

/// Moment of inertia, used for abstracting over 2D/3D inertia tensors
pub trait Inertia: Mul<Self, Output = Self> + Copy {
    /// Orientation type for rotating the inertia to create a world space inertia tensor
    type Orientation;
    /// Return the infinite inertia
    fn infinite() -> Self;
    /// Compute the inverse of the inertia
    fn invert(&self) -> Self;
    /// Compute the inertia tensor in the bodies given orientation
    fn tensor(&self, orientation: &Self::Orientation) -> Self;
}

impl Inertia for Real {
    type Orientation = Basis2<Real>;

    fn invert(&self) -> Self {
        if *self == 0. || self.is_infinite() {
            0.
        } else {
            1. / *self
        }
    }

    fn tensor(&self, _: &Basis2<Real>) -> Self {
        *self
    }

    fn infinite() -> Self {
        std::f64::INFINITY as Real
    }
}

impl Inertia for Matrix3<Real> {
    type Orientation = Quaternion<Real>;

    fn invert(&self) -> Self {
        if self.x.x.is_infinite() {
            Matrix3::zero()
        } else {
            SquareMatrix::invert(self).unwrap_or(Matrix3::zero())
        }
    }

    fn tensor(&self, orientation: &Quaternion<Real>) -> Self {
        let mat3 = Matrix3::from(*orientation);
        mat3 * (*self * mat3.transpose())
    }

    fn infinite() -> Self {
        Matrix3::from_value(std::f64::INFINITY as Real)
    }
}

impl<I> Mass<I>
where
    I: Inertia,
{
    /// Create new mass object
    pub fn new(mass: Real) -> Self {
        Self::new_with_inertia(mass, I::infinite())
    }

    /// Create new infinite mass object
    pub fn infinite() -> Self {
        Self::new_with_inertia(std::f64::INFINITY as Real, I::infinite())
    }

    /// Create new mass object with given inertia
    pub fn new_with_inertia(mass: Real, inertia: I) -> Self {
        let inverse_mass = if mass.is_infinite() { 0. } else { 1. / mass };
        let inverse_inertia = inertia.invert();
        Mass {
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
        }
    }

    /// Compute mass from the given volume shape and material
    pub fn from_volume_and_material<V>(volume: &V, material: &Material) -> Self
    where
        V: Volume<I>,
    {
        volume.get_mass(material)
    }

    /// Get mass
    pub fn mass(&self) -> Real {
        self.mass
    }

    /// Get inverse mass
    pub fn inverse_mass(&self) -> Real {
        self.inverse_mass
    }

    /// Get inertia in local space
    pub fn local_inertia(&self) -> I {
        self.inertia
    }

    /// Get inertia tensor in world space
    ///
    /// ### Parameters:
    ///
    /// - `orientation`: The current orientation of the body
    pub fn world_inertia(&self, orientation: &I::Orientation) -> I {
        self.inertia.tensor(orientation)
    }

    /// Get inverse inertia in local space
    pub fn local_inverse_inertia(&self) -> I {
        self.inverse_inertia
    }

    /// Get inverse inertia tensor in world space
    ///
    /// ### Parameters:
    ///
    /// - `orientation`: The current orientation of the body
    pub fn world_inverse_inertia(&self, orientation: &I::Orientation) -> I {
        self.inverse_inertia.tensor(orientation)
    }
}
