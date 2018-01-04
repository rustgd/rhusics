//! 2D structures for physics

pub use collide::prelude2d::*;
pub use physics::{resolve_contact, Inertia, Material, RigidBody, Volume};

use cgmath::Vector2;

use super::{ForceAccumulator, Mass, Velocity};

/// 2D velocity
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type Velocity2<S> = Velocity<Vector2<S>, S>;

/// 2D mass
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64) (also `Inertia` type for 2D)
pub type Mass2<S> = Mass<S, S>;

/// 2D force accumulator
///
/// ### Type parameters:
///
/// - `S`: Scalar type (f32 or f64)
pub type ForceAccumulator2<S> = ForceAccumulator<Vector2<S>, S>;
