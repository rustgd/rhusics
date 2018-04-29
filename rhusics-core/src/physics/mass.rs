use std;
use std::ops::Mul;

use cgmath::{BaseFloat, Basis2, Matrix, Matrix3, Quaternion, SquareMatrix, Zero};

use super::{Material, Volume};

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
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Mass<S, I> {
    mass: S,
    inverse_mass: S,

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

impl Inertia for f32 {
    type Orientation = Basis2<f32>;

    fn infinite() -> Self {
        std::f32::INFINITY
    }

    fn invert(&self) -> Self {
        if *self == 0. || self.is_infinite() {
            0.
        } else {
            1. / *self
        }
    }

    fn tensor(&self, _: &Basis2<f32>) -> Self {
        *self
    }
}

impl Inertia for f64 {
    type Orientation = Basis2<f64>;

    fn invert(&self) -> Self {
        if *self == 0. || self.is_infinite() {
            0.
        } else {
            1. / *self
        }
    }

    fn tensor(&self, _: &Basis2<f64>) -> Self {
        *self
    }

    fn infinite() -> Self {
        std::f64::INFINITY
    }
}

impl<S> Inertia for Matrix3<S>
where
    S: BaseFloat,
{
    type Orientation = Quaternion<S>;

    fn invert(&self) -> Self {
        if self.x.x.is_infinite() {
            Matrix3::zero()
        } else {
            SquareMatrix::invert(self).unwrap_or_else(Matrix3::zero)
        }
    }

    fn tensor(&self, orientation: &Quaternion<S>) -> Self {
        let mat3 = Matrix3::from(*orientation);
        mat3 * (*self * mat3.transpose())
    }

    fn infinite() -> Self {
        Matrix3::from_value(S::infinity())
    }
}

impl<S, I> Mass<S, I>
where
    S: BaseFloat,
    I: Inertia,
{
    /// Create new mass object
    pub fn new(mass: S) -> Self {
        Self::new_with_inertia(mass, I::infinite())
    }

    /// Create new infinite mass object
    pub fn infinite() -> Self {
        Self::new_with_inertia(S::infinity(), I::infinite())
    }

    /// Create new mass object with given inertia
    pub fn new_with_inertia(mass: S, inertia: I) -> Self {
        let inverse_mass = if mass.is_infinite() {
            S::zero()
        } else {
            S::one() / mass
        };
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
        V: Volume<S, I>,
    {
        volume.get_mass(material)
    }

    /// Get mass
    pub fn mass(&self) -> S {
        self.mass
    }

    /// Get inverse mass
    pub fn inverse_mass(&self) -> S {
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
