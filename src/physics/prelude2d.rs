//! 2D structures for physics

pub use collide::prelude2d::*;
pub use physics::{linear_resolve_contact, Mass};

use cgmath::{Basis2, Point2, Vector2};

use super::{LinearResolveData, Velocity};
use Real;

/// 2D velocity
pub type Velocity2 = Velocity<Vector2<Real>>;

/// 2D linear contact resolution data
pub type LinearResolveData2<'a> = LinearResolveData<'a, Point2<Real>, Basis2<Real>>;
