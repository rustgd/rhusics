//! 2D structures for physics

pub use collide::prelude2d::*;
pub use physics::{resolve_contact, Inertia, Material, RigidBody, Volume};

use cgmath::Vector2;

use super::{ForceAccumulator, Mass, Velocity};
use Real;

/// 2D velocity
pub type Velocity2 = Velocity<Vector2<Real>, Real>;

/// 2D mass
pub type Mass2 = Mass<Real>;

/// 2D force accumulator
pub type ForceAccumulator2 = ForceAccumulator<Vector2<Real>, Real>;
