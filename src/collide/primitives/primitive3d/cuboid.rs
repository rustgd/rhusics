use cgmath::{Point3, Vector3};
use cgmath::prelude::*;
use collision::{Ray3, Aabb3};
use collision::prelude::*;

use {Pose, Real};

/// Cuboid primitive.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone)]
pub struct Cuboid {
    /// Dimensions of the box
    pub dim: Vector3<Real>,
    half_dim: Vector3<Real>,
    corners: Vec<Point3<Real>>,
}

impl Cuboid {
    /// Create a new rectangle primitive from component dimensions
    pub fn new(dim_x: Real, dim_y: Real, dim_z: Real) -> Self {
        Self::new_impl(Vector3::new(dim_x, dim_y, dim_z))
    }

    /// Create a new rectangle primitive from a vector of component dimensions
    pub fn new_impl(dim: Vector3<Real>) -> Self {
        Self {
            dim,
            half_dim: dim / 2.,
            corners: Self::generate_corners(&dim),
        }
    }

    /// Get aabb for the cuboid
    #[inline]
    pub fn get_bound(&self) -> Aabb3<Real> {
        Aabb3::new(
            Point3::from_vec(-self.half_dim),
            Point3::from_vec(self.half_dim),
        )
    }

    /// Support function
    #[inline]
    pub fn get_far_point<T>(&self, direction: &Vector3<Real>, transform: &T) -> Point3<Real>
    where
        T: Pose<Point3<Real>>,
    {
        ::util::get_max_point(&self.corners, direction, transform)
    }

    fn generate_corners(dimensions: &Vector3<Real>) -> Vec<Point3<Real>> {
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

impl<T> Discrete<(Ray3<Real>, T)> for Cuboid
where
    T: Pose<Point3<Real>>,
{
    fn intersects(&self, &(ref ray, ref transform): &(Ray3<Real>, T)) -> bool {
        let inverse_transform = transform.inverse_transform().unwrap();
        let n_ray = Ray3::new(
            inverse_transform.transform_point(ray.origin),
            inverse_transform.transform_vector(ray.direction),
        );
        self.get_bound().intersects(&n_ray)
    }
}

impl<T> Continuous<(Ray3<Real>, T)> for Cuboid
where
    T: Pose<Point3<Real>>,
{
    type Result = Point3<Real>;

    fn intersection(&self, &(ref ray, ref transform): &(Ray3<Real>, T)) -> Option<Point3<Real>> {
        let inverse_transform = transform.inverse_transform().unwrap();
        let n_ray = Ray3::new(
            inverse_transform.transform_point(ray.origin),
            inverse_transform.transform_vector(ray.direction),
        );
        self.get_bound().intersection(&n_ray)
    }
}
