//! 2D structures for physics

pub use collide::prelude2d::*;
pub use physics::{linear_resolve_contact, RigidBody, Material};

use cgmath::{Basis2, Point2, Vector2};

use super::{LinearResolveData, Mass, Velocity, ForceAccumulator};
use Real;

/// 2D velocity
pub type Velocity2 = Velocity<Vector2<Real>>;

/// 2D mass
pub type Mass2 = Mass<Real>;

/// 2D linear contact resolution data
pub type LinearResolveData2<'a> = LinearResolveData<'a, Point2<Real>, Basis2<Real>>;

/// 2D force accumulator
pub type ForceAccumulator2 = ForceAccumulator<Vector2<Real>>;
