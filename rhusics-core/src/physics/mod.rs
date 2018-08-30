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

mod force;
mod mass;
mod util;
mod velocity;
mod volumes;

use cgmath::{BaseFloat, VectorSpace};

/// Global parameters for the physics world
pub struct WorldParameters<V, S> {
    gravity: V,
    damping: S,
}

impl<V, S> Default for WorldParameters<V, S>
where
    V: VectorSpace,
    S: BaseFloat,
{
    fn default() -> Self {
        WorldParameters::new(V::zero())
    }
}

impl<V, S> WorldParameters<V, S>
where
    V: VectorSpace,
    S: BaseFloat,
{
    /// Setup global parameters for the physics world
    pub fn new(gravity: V) -> Self {
        WorldParameters {
            gravity,
            damping: S::from(0.99).unwrap(),
        }
    }

    /// Set global damping, can be overriden by individual physical entities
    pub fn with_damping(mut self, damping: S) -> Self {
        self.damping = damping;
        self
    }

    /// Get gravity
    pub fn gravity(&self) -> V {
        self.gravity
    }

    /// Get global damping
    pub fn damping(&self) -> S {
        self.damping
    }

    /// Get damping for a specific physics entity
    pub fn entity_damping(&self, body: Option<S>) -> S {
        body.unwrap_or(self.damping)
    }
}

/// Physics material
///
/// Used to describe physical properties of physical entities, such as density and restitution.
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

/// Physical entity
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PhysicalEntity<S> {
    material: Material,
    gravity_scale: S,
    damping: Option<S>,
    active: bool,
}

impl<S> Default for PhysicalEntity<S>
where
    S: BaseFloat,
{
    fn default() -> Self {
        PhysicalEntity::new(Material::default())
    }
}

impl<S> PhysicalEntity<S>
where
    S: BaseFloat,
{
    /// Create new physical entity
    ///
    /// ## Parameters:
    ///
    /// - material: physical material (`Material`)
    pub fn new(material: Material) -> Self {
        Self {
            material,
            gravity_scale: S::one(),
            damping: None,
            active: true,
        }
    }

    /// Set the amount that gravity will affect this entity
    /// The gravity constant is set globally for the physics world.
    pub fn with_gravity_scale(mut self, gravity_scale: S) -> Self {
        self.gravity_scale = gravity_scale;
        self
    }

    /// Override the velocity damping for the entity
    /// The physics world control have a global damping set which is overriden by this.
    pub fn with_damping(mut self, damping: S) -> Self {
        self.damping = Some(damping);
        self
    }

    /// Get material
    pub fn material(&self) -> &Material {
        &self.material
    }

    /// Get gravity scale
    pub fn gravity_scale(&self) -> S {
        self.gravity_scale
    }

    /// Get entity specific damping
    pub fn damping(&self) -> Option<S> {
        self.damping
    }

    /// Is entity active ?
    pub fn active(&self) -> bool {
        self.active
    }

    /// Set entity to active, meaning physics will act on it
    pub fn activate(&mut self) {
        self.active = true;
    }

    /// Set entity to inactive, meaning physics will not act on it
    pub fn deactivate(&mut self) {
        self.active = false;
    }
}
