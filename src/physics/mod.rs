//! Physics related functionality
//!

pub use self::simple::{linear_resolve_contact, LinearResolveData};

pub mod prelude2d;
pub mod prelude3d;

use cgmath::{VectorSpace, Zero};

use Real;

mod simple;
mod volumes;

/// Velocity
#[derive(Debug, Clone)]
pub struct Velocity<V>
where
    V: Clone,
{
    linear: V,
    angular: V,
}

impl<V> Velocity<V>
where
    V: Clone + Zero,
{
    /// Create new velocity object
    pub fn from_linear(linear: V) -> Self {
        Self {
            linear,
            angular: V::zero(),
        }
    }

    /// Set linear velocity
    pub fn set_linear(&mut self, linear: V) {
        self.linear = linear;
    }

    /// Get linear velocity
    pub fn linear(&self) -> &V {
        &self.linear
    }
}

/// Mass
#[derive(Debug)]
pub struct Mass<I> {
    mass: Real,
    inverse_mass: Real,

    inertia: I, // TODO: 3d inertia
    inverse_inertia: I,
}

impl<I> Mass<I>
where
    I: Zero + Copy,
{
    /// Create new mass object
    pub fn new(mass: Real) -> Self {
        Self::new_with_inertia(mass, I::zero())
    }

    /// Create new mass object with inertia
    pub fn new_with_inertia(mass: Real, inertia: I) -> Self {
        let inverse_mass = if mass.is_infinite() { 0. } else { 1. / mass };
        let inverse_inertia = inertia; // FIXME
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
}

/// Physics material
pub struct Material {
    density: Real,
    restitution: Real,
}

impl Default for Material {
    fn default() -> Self {
        Material::new(1., 0.)
    }
}

impl Material {
    /// Rock
    pub const ROCK: Material = Material {
        density: 0.6,
        restitution: 0.1,
    };
    /// Wood
    pub const WOOD: Material = Material {
        density: 0.3,
        restitution: 0.2,
    };
    /// Metal
    pub const METAL: Material = Material {
        density: 1.2,
        restitution: 0.05,
    };
    /// Bouncy Ball
    pub const BOUNCY_BALL: Material = Material {
        density: 0.3,
        restitution: 0.8,
    };
    /// Super Ball
    pub const SUPER_BALL: Material = Material {
        density: 0.3,
        restitution: 0.95,
    };
    /// Pillow
    pub const PILLOW: Material = Material {
        density: 0.1,
        restitution: 0.2,
    };
    /// Static
    pub const STATIC: Material = Material {
        density: 0.0,
        restitution: 0.4,
    };

    /// Create new material
    pub fn new(density: Real, restitution: Real) -> Self {
        Self {
            density,
            restitution,
        }
    }

    /// Get density
    pub fn density(&self) -> Real {
        self.density
    }

    /// Get restitution
    pub fn restitution(&self) -> Real {
        self.restitution
    }
}

/// Rigid body
pub struct RigidBody {
    material: Material,
    gravity_scale: Real,
}

/// Describe a shape with volume
pub trait Volume<I> {
    /// Compute the mass of the shape based on its material
    fn get_mass(&self, material: &Material) -> Mass<I>;
}

impl Default for RigidBody {
    fn default() -> Self {
        RigidBody::new(Material::default(), 1.0)
    }
}

impl RigidBody {
    /// Create new rigid body
    pub fn new(material: Material, gravity_scale: Real) -> Self {
        Self {
            material,
            gravity_scale,
        }
    }

    /// Get material
    pub fn material(&self) -> &Material {
        &self.material
    }

    /// Get gravity scale
    pub fn gravity_scale(&self) -> Real {
        self.gravity_scale
    }
}

/// Force accumulator for a rigid body
pub struct ForceAccumulator<V> {
    force: V,
}

impl<V> ForceAccumulator<V>
where
    V: VectorSpace<Scalar = Real> + Zero,
{
    /// Create a new force accumulator
    pub fn new() -> Self {
        Self { force: V::zero() }
    }

    /// Add a force vector to the accumulator
    pub fn add(&mut self, force: V) {
        self.force = self.force + force;
    }

    /// Consume the force vector
    pub fn consume(&mut self) -> V {
        let v = self.force.clone();
        self.force = V::zero();
        v
    }
}
