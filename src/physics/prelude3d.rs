//! 3D structures for physics

pub use collide::prelude3d::*;
pub use physics::{linear_resolve_contact, Inertia, Material, RigidBody, Volume};

use cgmath::{Matrix3, Vector3};

use super::{ForceAccumulator, Mass, Velocity};
use Real;

/// 3D velocity
pub type Velocity3 = Velocity<Vector3<Real>, Vector3<Real>>;

/// 3D mass
pub type Mass3 = Mass<Matrix3<Real>>;

/// 3D force accumulator
pub type ForceAccumulator3 = ForceAccumulator<Vector3<Real>>;
