//! # Rhusics physics library
//!
//! A physics library, primarily built to be used with
//! [`specs`](https://github.com/slide-rs/specs/).
//! Uses [`cgmath`](https://github.com/brendanzab/cgmath/) for all computation.
//!
//! Features:
//!
//! * Collision detection primitives for 2D and 3D:
//!   * 2D
//!     - Circle
//!     - Rectangle
//!     - Convex polygon with any number of vertices
//!   * 3D
//!     - Sphere
//!     - Cuboid
//!     - Convex polyhedron with any number of vertices
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
//!
//! # Examples
//!
//! ```rust
//! extern crate rhusics;
//! extern crate cgmath;
//! extern crate specs;
//!
//! use cgmath::{Transform, Rotation2, Rad, Point2};
//! use specs::{World, RunNow};
//!
//! use rhusics::collide2d::{CollisionShape2, BasicCollisionSystem2, BodyPose2,
//!                          BroadBruteForce2, GJK2, world_register, Rectangle,
//!                          Contacts2, CollisionStrategy};
//!
//! pub fn main() {
//!     let mut world = World::new();
//!     world_register::<BodyPose2>(&mut world);
//!
//!     world
//!         .create_entity()
//!         .with(CollisionShape2::<BodyPose2>::new_simple(
//!             CollisionStrategy::FullResolution,
//!             Rectangle::new(10., 10.).into(),
//!         ))
//!         .with(BodyPose2::one());
//!
//!     world
//!         .create_entity()
//!         .with(CollisionShape2::<BodyPose2>::new_simple(
//!             CollisionStrategy::FullResolution,
//!             Rectangle::new(10., 10.).into(),
//!         ))
//!         .with(BodyPose2::new(
//!             Point2::new(3., 2.),
//!             Rotation2::from_angle(Rad(0.)),
//!         ));
//!
//!     let mut system = BasicCollisionSystem2::<BodyPose2>::new()
//!         .with_broad_phase(BroadBruteForce2::default())
//!         .with_narrow_phase(GJK2::new());
//!     system.run_now(&world.res);
//!     println!("Contacts: {:?}", *world.read_resource::<Contacts2>());
//! }
//! ```

#![deny(missing_docs, missing_debug_implementations, trivial_casts,
unsafe_code, unstable_features, unused_import_braces, unused_qualifications)]

extern crate cgmath;
extern crate collision;
extern crate specs;
extern crate rand;

#[cfg(test)]
#[macro_use]
extern crate assert_approx_eq;

pub mod collide;
pub mod collide2d;
pub mod collide3d;

use cgmath::prelude::*;

mod util;
mod ecs;

#[cfg(not(feature = "double"))]
pub(crate) type Real = f32;

#[cfg(feature = "double")]
pub(crate) type Real = f64;

/// Trait bound used for transforms throughout the library
pub trait Pose<P>: Transform<P>
where
    P: EuclideanSpace<Scalar = Real>,
{
    /// The rotational data type used by the concrete implementation
    type Rotation: Rotation<P>;

    /// Borrows the position attribute
    fn position(&self) -> &P;

    /// Borrows the rotation attribute
    fn rotation(&self) -> &Self::Rotation;

    /// Checks to see if the transform is dirty. Used by the collision system to see if bounds need
    /// to be recalculated.
    fn dirty(&self) -> bool;

    /// Borrows the inverse rotation attribute
    fn inverse_rotation(&self) -> &Self::Rotation;
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
            self.rotation.rotate_point(self.position) *
                -P::Scalar::one(),
            self.inverse_rotation,
        ))
    }
}

impl<P, R> Pose<P> for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
{
    type Rotation = R;

    fn position(&self) -> &P {
        &self.position
    }

    fn rotation(&self) -> &R {
        &self.rotation
    }

    fn dirty(&self) -> bool {
        self.dirty
    }

    fn inverse_rotation(&self) -> &R {
        &self.inverse_rotation
    }
}
