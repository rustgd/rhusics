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
    missing_docs,
    trivial_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]
#![allow(unknown_lints, type_complexity, new_without_default_derive)]

extern crate cgmath;
extern crate collision;
extern crate failure;
extern crate rhusics_core as core;
extern crate shred;
extern crate shrev;
extern crate specs;

extern crate shred_derive;

#[cfg(feature = "serializable")]
#[macro_use]
extern crate serde;

pub use collide::{BasicCollisionSystem, SpatialCollisionSystem, SpatialSortingSystem};
pub use physics::{
    setup_dispatch, ContactResolutionSystem, CurrentFrameUpdateSystem, DeltaTime,
    NextFrameSetupSystem, PhysicalEntityCreationError, PhysicalEntityParts, WithPhysics,
};

pub mod collide2d;
pub mod collide3d;
pub mod physics2d;
pub mod physics3d;

mod collide;
mod physics;
