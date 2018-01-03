//! 3D structures for physics

pub use collide::prelude3d::*;
pub use physics::{resolve_contact, Inertia, Material, RigidBody, Volume};

use cgmath::{Matrix3, Vector3};

use super::{ForceAccumulator, Mass, Velocity};

/// 3D velocity
pub type Velocity3<S> = Velocity<Vector3<S>, Vector3<S>>;

/// 3D mass
pub type Mass3<S> = Mass<S, Matrix3<S>>;

/// 3D force accumulator
pub type ForceAccumulator3<S> = ForceAccumulator<Vector3<S>, Vector3<S>>;
