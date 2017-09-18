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
use cgmath::prelude::*;
use collision::Aabb3;

use super::Primitive;
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
            Primitive3::Sphere(ref sphere) => sphere.get_far_point(direction, transform),

            Primitive3::Cuboid(ref b) => b.get_far_point(direction, transform),

            Primitive3::ConvexPolytope(ref c) => c.get_far_point(direction, transform),
        }
    }
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::{Point3, Quaternion, Rad, Rotation3, Vector3};

    use super::*;
    use BodyPose;

    // sphere
    #[test]
    fn test_sphere_far_1() {
        test_sphere(1., 0., 0., 10., 0., 0., 0.);
    }

    #[test]
    fn test_sphere_far_2() {
        test_sphere(
            1.,
            1.,
            1.,
            5.773502691896258,
            5.773502691896258,
            5.773502691896258,
            0.,
        );
    }

    #[test]
    fn test_sphere_far_3() {
        test_sphere(1., 0., 0., 10., 0., 0., -std::f64::consts::PI as Real / 4.);
    }

    #[test]
    fn test_sphere_far_4() {
        let sphere: Primitive3 = Sphere::new(10.).into();
        let direction = Vector3::new(1., 0., 0.);
        let transform: BodyPose<Point3<Real>, Quaternion<Real>> =
            BodyPose::new(Point3::new(0., 10., 0.), Quaternion::from_angle_z(Rad(0.)));
        let point = sphere.get_far_point(&direction, &transform);
        assert_eq!(Point3::new(10., 10., 0.), point);
    }

    #[test]
    fn test_sphere_bound() {
        let sphere: Primitive3 = Sphere::new(10.).into();
        assert_eq!(bound(-10., -10., -10., 10., 10., 10.), sphere.get_bound())
    }

    fn test_sphere(dx: Real, dy: Real, dz: Real, px: Real, py: Real, pz: Real, rot: Real) {
        let sphere: Primitive3 = Sphere::new(10.).into();
        let direction = Vector3::new(dx, dy, dz);
        let transform: BodyPose<Point3<Real>, Quaternion<Real>> =
            BodyPose::new(Point3::new(0., 0., 0.), Quaternion::from_angle_z(Rad(rot)));
        let point = sphere.get_far_point(&direction, &transform);
        assert_approx_eq!(px, point.x);
        assert_approx_eq!(py, point.y);
        assert_approx_eq!(pz, point.z);
    }

    // box
    // not testing far point as ::util::get_max_point is rigorously tested
    #[test]
    fn test_rectangle_bound() {
        let r: Primitive3 = Cuboid::new(10., 10., 10.).into();
        assert_eq!(bound(-5., -5., -5., 5., 5., 5.), r.get_bound())
    }

    // convex polyhedron
    // not testing bound as ::util::get_bound is fairly well tested
    // not testing far point as ::util::get_max_point is rigorously tested

    // util
    fn bound(
        min_x: Real,
        min_y: Real,
        min_z: Real,
        max_x: Real,
        max_y: Real,
        max_z: Real,
    ) -> Aabb3<Real> {
        Aabb3::new(
            Point3::new(min_x, min_y, min_z),
            Point3::new(max_x, max_y, max_z),
        )
    }
}
