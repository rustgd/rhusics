//! # Rhusics physics library
//!
//! A physics library.
//! Uses [`cgmath`](https://github.com/brendanzab/cgmath/) for all computation.
//!
//! Features:
//!
//! * Two different broad phase collision detection implementations:
//!   * Brute force
//!   * Sweep and Prune
//! * Narrow phase collision detection using GJK, and optionally EPA for full contact information
//! * Functions for collision detection working on user supplied transform, and
//!    [`CollisionShape`](collide/struct.CollisionShape.html) components.
//!    Can optionally use broad and/or narrow phase detection.
//!    Library supplies a transform implementation [`BodyPose`](struct.BodyPose.html) for
//!    convenience.
//! * Uses single precision as default, can be changed to double precision with the `double`
//!   feature.
//! * Has support for doing spatial sort/collision detection using the collision-rs DBVT.
//! * Support for doing broad phase using the collision-rs DBVT.
//! * Has support for all primitives in collision-rs
//!

#![deny(
    missing_docs, trivial_casts, unsafe_code, unstable_features, unused_import_braces,
    unused_qualifications
)]
#![allow(unknown_lints, type_complexity, borrowed_box)]

extern crate cgmath;
extern crate collision;
extern crate rhusics_transform;

#[cfg(feature = "specs")]
extern crate specs;

#[cfg(test)]
#[macro_use]
extern crate approx;

#[cfg(feature = "serde")]
#[macro_use]
extern crate serde;

pub use body_pose::BodyPose;
pub use collide::broad::{BroadPhase, BruteForce, SweepAndPrune2, SweepAndPrune3};
pub use collide::narrow::NarrowPhase;
pub use collide::{
    basic_collide, tree_collide, Collider, CollisionData, CollisionMode, CollisionShape,
    CollisionStrategy, Contact, ContactEvent, GetId, Primitive,
};
pub use physics::simple::{next_frame_integration, next_frame_pose};
pub use physics::{
    resolve_contact, ApplyAngular, ForceAccumulator, Inertia, Mass, Material, PartialCrossProduct,
    PhysicalEntity, ResolveData, SingleChangeSet, Velocity, Volume, WorldParameters,
};
pub use rhusics_transform::{PhysicsTime, Pose};

pub mod collide2d;
pub mod collide3d;
pub mod physics2d;
pub mod physics3d;

mod body_pose;
mod collide;
#[cfg(feature = "specs")]
mod ecs;
mod physics;

/// Wrapper for data computed for the next frame
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct NextFrame<T> {
    /// Wrapped value
    pub value: T,
}
