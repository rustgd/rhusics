//! Physics related functionality
//!

pub use self::force::ForceAccumulator;
pub use self::mass::{Inertia, Mass};
pub use self::resolution::{resolve_contact, ResolveData, SingleChangeSet};
pub use self::util::PartialCrossProduct;
pub use self::velocity::{ApplyAngular, Velocity};
pub use self::volumes::Volume;

pub mod simple;

mod resolution;

mod volumes;
mod mass;
mod velocity;
mod force;
mod util;

use cgmath::BaseFloat;

/// Physics material
///
/// Used to describe physical properties of rigid bodies, such as density and restitution.
///
/// The default material has density 1, such that only the volume affects its mass, and restitution
/// 1, such that all energy is preserved in collisions.
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Material {
    density: f32,
    restitution: f32,
}

impl Default for Material {
    fn default() -> Self {
        Material::new(1., 1.)
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
    pub fn new(density: f32, restitution: f32) -> Self {
        Self {
            density,
            restitution,
        }
    }

    /// Get density
    pub fn density<S>(&self) -> S
    where
        S: BaseFloat,
    {
        S::from(self.density).unwrap()
    }

    /// Get restitution
    pub fn restitution<S>(&self) -> S
    where
        S: BaseFloat,
    {
        S::from(self.restitution).unwrap()
    }
}

/// Rigid body
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RigidBody<S> {
    material: Material,
    gravity_scale: S,
}

impl<S> Default for RigidBody<S>
where
    S: BaseFloat,
{
    fn default() -> Self {
        RigidBody::new(Material::default(), S::one())
    }
}

impl<S> RigidBody<S>
where
    S: BaseFloat,
{
    /// Create new rigid body
    pub fn new(material: Material, gravity_scale: S) -> Self {
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
    pub fn gravity_scale(&self) -> S {
        self.gravity_scale
    }
}
