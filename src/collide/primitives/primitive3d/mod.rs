//! Collision primitives for 3D.
//!
//! These are the building blocks of all collision detection for 3D.
//! The enum [`Primitive3D`](enum.Primitive3D.html) is the main type, that implements the
//! [`Primitive`](../trait.Primitive.html) trait.
//!
//! All primitive types have `Into` implementations to turn them into enum variants for
//! [`Primitive3D`](enum.Primitive3D.html).
//!
//! # Example
//!
//! ```
//! # use rhusics::collide::primitives::primitive3d::*;
//! use rhusics::collide::Primitive;
//! let p : Primitive3 = Cuboid::new(10., 34., 22.).into();
//! p.get_bound();
//! ```

pub use self::cuboid::Cuboid;
pub use self::polytope::ConvexPolytope;
pub use self::sphere::Sphere;

use cgmath::{Point3, Vector3};
use collision::Aabb3;

use super::{Primitive, HasAABB, SupportFunction};
use {Pose, Real};

mod polytope;
mod sphere;
mod cuboid;

/// Base enum for all 3D primitives
#[derive(Debug, Clone)]
pub enum Primitive3 {
    /// Sphere variant
    Sphere(Sphere),

    /// Cuboid variant
    Cuboid(Cuboid),

    /// Convex polyhedron variant
    ConvexPolytope(ConvexPolytope),
    // TODO: more primitives
}

impl Into<Primitive3> for Sphere {
    fn into(self) -> Primitive3 {
        Primitive3::Sphere(self)
    }
}

impl Into<Primitive3> for Cuboid {
    fn into(self) -> Primitive3 {
        Primitive3::Cuboid(self)
    }
}

impl Into<Primitive3> for ConvexPolytope {
    fn into(self) -> Primitive3 {
        Primitive3::ConvexPolytope(self)
    }
}

impl Primitive for Primitive3 {
    type Vector = Vector3<Real>;
    type Point = Point3<Real>;
    type Aabb = Aabb3<Real>;

    fn get_bound(&self) -> Aabb3<Real> {
        match *self {
            Primitive3::Sphere(ref sphere) => sphere.get_bound(),

            Primitive3::Cuboid(ref b) => b.get_bound(),

            Primitive3::ConvexPolytope(ref c) => c.get_bound(),
        }
    }

    fn get_far_point<T>(&self, direction: &Vector3<Real>, transform: &T) -> Point3<Real>
    where
        T: Pose<Point3<Real>>,
    {
        match *self {
            Primitive3::Sphere(ref sphere) => sphere.support_point(direction, transform),

            Primitive3::Cuboid(ref b) => b.support_point(direction, transform),

            Primitive3::ConvexPolytope(ref c) => c.support_point(direction, transform),
        }
    }
}
