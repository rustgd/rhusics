//! # Rhusics physics library
//!
//! A physics library, primarily built to be used with
//! [`specs`](https://github.com/slide-rs/specs/).
//! Uses [`cgmath`](https://github.com/brendanzab/cgmath/) for all computation.
//!
//! Features:
//!
//! * Two different broad phase collision detection implementations:
//!   * Brute force
//!   * Sweep and Prune
//! * Narrow phase collision detection using GJK, and optionally EPA for full contact information
//! * [`specs::System`](https://docs.rs/specs/0.9.5/specs/trait.System.html) for collision
//!    detection working on user supplied transform, and
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
//! # Examples
//!
//! See the `examples/` directory for examples.

#![deny(missing_docs, trivial_casts, unsafe_code, unstable_features, unused_import_braces,
       unused_qualifications)]

extern crate cgmath;
extern crate collision;
#[cfg(feature = "ecs")]
extern crate shrev;
#[cfg(feature = "ecs")]
extern crate specs;

#[cfg(test)]
#[macro_use]
extern crate approx;

pub mod collide;
#[cfg(feature = "ecs")]
pub mod ecs;

use cgmath::prelude::*;
use collision::prelude::*;

#[cfg(not(feature = "double"))]
pub(crate) type Real = f32;

#[cfg(feature = "double")]
pub(crate) type Real = f64;

/// Wrapper for data computed for the next frame
#[derive(Clone, Debug)]
pub struct NextFrame<T> {
    /// Wrapped value
    pub value: T,
}

/// Transform that implements [`Pose`](trait.Pose.html), and can be used as the transform
/// component throughout the library.
#[derive(Clone, Debug)]
pub struct BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
{
    dirty: bool,
    position: P,
    rotation: R,
    inverse_rotation: R,
}

impl<P, R> BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
{
    /// Create a new [`BodyPose`](struct.BodyPose.html) with initial state given by the supplied
    /// position and rotation.
    pub fn new(position: P, rotation: R) -> Self {
        Self {
            dirty: true,
            position,
            inverse_rotation: rotation.invert(),
            rotation,
        }
    }

    /// Set the rotation. Will also compute the inverse rotation. Sets the dirty flag.
    pub fn set_rotation(&mut self, rotation: R) {
        self.rotation = rotation;
        self.inverse_rotation = self.rotation.invert();
        self.dirty = true;
    }

    /// Set the position. Sets the dirty flag.
    pub fn set_position(&mut self, position: P) {
        self.position = position;
        self.dirty = true;
    }

    /// Borrows the position attribute
    pub fn position(&self) -> &P {
        &self.position
    }

    /// Borrows the rotation attribute
    pub fn rotation(&self) -> &R {
        &self.rotation
    }

    /// Clear the dirty flag
    pub fn clear(&mut self) {
        self.dirty = false;
    }
}

impl<P, R> Transform<P> for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
{
    fn one() -> Self {
        Self::new(P::from_value(0.), R::one())
    }

    fn look_at(eye: P, center: P, up: P::Diff) -> Self {
        let rot = R::look_at(center - eye, up);
        let disp = rot.rotate_vector(P::origin() - eye);
        Self::new(P::from_vec(disp), rot)
    }

    fn transform_vector(&self, vec: P::Diff) -> P::Diff {
        self.rotation.rotate_vector(vec)
    }

    fn transform_point(&self, point: P) -> P {
        self.rotation.rotate_point(point) + self.position.to_vec()
    }

    fn concat(&self, other: &Self) -> Self {
        Self::new(
            self.position + self.rotation.rotate_point(other.position).to_vec(),
            self.rotation * other.rotation,
        )
    }

    fn inverse_transform(&self) -> Option<Self> {
        Some(Self::new(
            self.rotation.rotate_point(self.position) * -P::Scalar::one(),
            self.inverse_rotation,
        ))
    }
}

impl<P, R> TranslationInterpolate<P::Scalar> for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    P::Diff: VectorSpace + InnerSpace,
    R: Rotation<P> + Clone,
{
    fn translation_interpolate(&self, other: &Self, amount: P::Scalar) -> Self {
        BodyPose::new(
            P::from_vec(self.position.to_vec().lerp(other.position.to_vec(), amount)),
            other.rotation.clone(),
        )
    }
}

impl<P, R> Interpolate<P::Scalar> for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    P::Diff: VectorSpace + InnerSpace,
    R: Rotation<P> + Interpolate<P::Scalar>,
{
    fn interpolate(&self, other: &Self, amount: P::Scalar) -> Self {
        BodyPose::new(
            P::from_vec(self.position.to_vec().lerp(other.position.to_vec(), amount)),
            self.rotation.interpolate(&other.rotation, amount),
        )
    }
}
