//! 3D structures for physics

pub use collide::prelude3d::*;
pub use physics::{linear_resolve_contact, Inertia, Material, RigidBody, Volume};

use cgmath::{Matrix3, Point3, Quaternion, Vector3};

use super::{ForceAccumulator, LinearResolveData, Mass, Velocity};
use Real;

/// 3D velocity
pub type Velocity3 = Velocity<Vector3<Real>>;

/// 3D mass
pub type Mass3 = Mass<Real>; // FIXME: Use matrix3 when ready

/// 3D linear contact resolution data
pub type LinearResolveData3<'a> = LinearResolveData<
    'a,
    Point3<Real>,
    Quaternion<Real>,
    Matrix3<Real>,
>;

/// 3D force accumulator
pub type ForceAccumulator3 = ForceAccumulator<Vector3<Real>>;
