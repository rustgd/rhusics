use cgmath::{Vector3, Point3};
use cgmath::prelude::*;
use collision::Aabb3;

use super::Primitive;
use {Pose, Real};

#[derive(Debug, Clone)]
pub struct Sphere {
    pub radius: Real,
}

impl Sphere {
    pub fn new(radius: Real) -> Self {
        Self { radius }
    }
}

#[derive(Debug, Clone)]
pub struct Box {
    pub dim: Vector3<Real>,
    half_dim: Vector3<Real>,
    corners: Vec<Point3<Real>>,
}

impl Box {
    pub fn new(dim_x: Real, dim_y: Real, dim_z: Real) -> Self {
        Self::new_impl(Vector3::new(dim_x, dim_y, dim_z))
    }

    pub fn new_impl(dim: Vector3<Real>) -> Self {
        Self {
            dim,
            half_dim: dim / 2.,
            corners: Self::generate_corners(&dim),
        }
    }

    pub fn generate_corners(dimensions: &Vector3<Real>) -> Vec<Point3<Real>> {
        let two = 2.;
        vec![
            Point3::new(dimensions.x, dimensions.y, dimensions.z) / two,
            Point3::new(-dimensions.x, dimensions.y, dimensions.z) / two,
            Point3::new(-dimensions.x, -dimensions.y, dimensions.z) / two,
            Point3::new(dimensions.x, -dimensions.y, dimensions.z) / two,
            Point3::new(dimensions.x, dimensions.y, -dimensions.z) / two,
            Point3::new(-dimensions.x, dimensions.y, -dimensions.z) / two,
            Point3::new(-dimensions.x, -dimensions.y, -dimensions.z) / two,
            Point3::new(dimensions.x, -dimensions.y, -dimensions.z) / two,
        ]
    }
}

#[derive(Debug, Clone)]
pub struct ConvexPolyhedron {
    pub vertices: Vec<Point3<Real>>,
}

#[derive(Debug, Clone)]
pub enum Primitive3D {
    Sphere(Sphere),
    Box(Box),
    ConvexPolyhedron(ConvexPolyhedron),
    // TODO: more primitives
}

impl Into<Primitive3D> for Sphere {
    fn into(self) -> Primitive3D {
        Primitive3D::Sphere(self)
    }
}

impl Into<Primitive3D> for Box {
    fn into(self) -> Primitive3D {
        Primitive3D::Box(self)
    }
}

impl Into<Primitive3D> for ConvexPolyhedron {
    fn into(self) -> Primitive3D {
        Primitive3D::ConvexPolyhedron(self)
    }
}

impl Primitive<Aabb3<Real>> for Primitive3D {
    fn get_bound(&self) -> Aabb3<Real> {
        match *self {
            Primitive3D::Sphere(ref sphere) => {
                Aabb3::new(
                    Point3::from_value(-sphere.radius),
                    Point3::from_value(sphere.radius),
                )
            }
            Primitive3D::Box(ref b) => {
                Aabb3::new(Point3::from_vec(-b.half_dim), Point3::from_vec(b.half_dim))
            }
            Primitive3D::ConvexPolyhedron(ref c) => ::util::get_bound(&c.vertices),
        }
    }

    fn get_far_point<T>(&self, direction: &Vector3<Real>, transform: &T) -> Point3<Real>
    where
        T: Pose<Point3<Real>>,
    {
        match *self {
            Primitive3D::Sphere(ref sphere) => {
                let direction = transform.inverse_rotation().rotate_vector(*direction);
                transform.position() + direction.normalize_to(sphere.radius)
            }
            Primitive3D::Box(ref b) => ::util::get_max_point(&b.corners, direction, transform),
            Primitive3D::ConvexPolyhedron(ref c) => {
                ::util::get_max_point(&c.vertices, direction, transform)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::{Point3, Vector3, Rotation3, Rad, Quaternion};

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
        test_sphere(
            1.,
            0.,
            0.,
            7.071067,
            7.0710683,
            0.,
            -std::f64::consts::PI as Real / 4.,
        );
    }

    #[test]
    fn test_sphere_far_4() {
        let sphere: Primitive3D = Sphere::new(10.).into();
        let direction = Vector3::new(1., 0., 0.);
        let transform: BodyPose<Point3<Real>, Quaternion<Real>> =
            BodyPose::new(Point3::new(0., 10., 0.), Quaternion::from_angle_z(Rad(0.)));
        let point = sphere.get_far_point(&direction, &transform);
        assert_eq!(Point3::new(10., 10., 0.), point);
    }

    #[test]
    fn test_sphere_bound() {
        let sphere: Primitive3D = Sphere::new(10.).into();
        assert_eq!(bound(-10., -10., -10., 10., 10., 10.), sphere.get_bound())
    }

    fn test_sphere(dx: Real, dy: Real, dz: Real, px: Real, py: Real, pz: Real, rot: Real) {
        let sphere: Primitive3D = Sphere::new(10.).into();
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
        let r: Primitive3D = Box::new(10., 10., 10.).into();
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
