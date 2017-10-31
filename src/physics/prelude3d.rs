//! 3D structures for physics

pub use collide::prelude3d::*;
pub use physics::{linear_resolve_contact, Mass};

use cgmath::{Point3, Quaternion, Vector3};

use super::{LinearResolveData, Velocity};
use Real;

/// 3D velocity
pub type Velocity3 = Velocity<Vector3<Real>>;

/// 3D linear contact resolution data
pub type LinearResolveData3<'a> = LinearResolveData<'a, Point3<Real>, Quaternion<Real>>;
