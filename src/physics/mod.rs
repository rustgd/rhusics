//! Physics related functionality
//!

pub use self::simple::{linear_resolve_contact, LinearResolveData};

pub mod prelude2d;
pub mod prelude3d;

use Real;

mod simple;

/// Velocity
#[derive(Debug, Clone)]
pub struct Velocity<V>
where
    V: Clone,
{
    /// Linear velocity
    pub linear: V,
}

/// Mass
#[derive(Debug)]
pub struct Mass {
    mass: Real,
    inverse_mass: Real,
}

impl Mass {
    /// Create new mass object
    pub fn new(mass: Real) -> Self {
        if mass.is_infinite() {
            Self {
                mass,
                inverse_mass: 0.,
            }
        } else {
            Self {
                mass,
                inverse_mass: 1. / mass,
            }
        }
    }
}
