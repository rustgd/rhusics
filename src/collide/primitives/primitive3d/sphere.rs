use cgmath::{Point3, Vector3};
use cgmath::prelude::*;
use collision::{Ray3, Aabb3};
use collision::prelude::*;

use {Pose, Real};

/// Sphere primitive
#[derive(Debug, Clone)]
pub struct Sphere {
    /// Radius of the sphere
    pub radius: Real,
}

impl Sphere {
    /// Create a new sphere primitive
    pub fn new(radius: Real) -> Self {
        Self { radius }
    }

    #[inline]
    /// Get AABB of sphere
    pub fn get_bound(&self) -> Aabb3<Real> {
        Aabb3::new(
            Point3::from_value(-self.radius),
            Point3::from_value(self.radius),
        )
    }

    #[inline]
    /// Support function
    pub fn get_far_point<T>(&self, direction: &Vector3<Real>, transform: &T) -> Point3<Real>
    where
        T: Pose<Point3<Real>>,
    {
        transform.position() + direction.normalize_to(self.radius)
    }
}

impl<T> Discrete<(Ray3<Real>, T)> for Sphere
where
    T: Pose<Point3<Real>>,
{
    fn intersects(&self, &(ref ray, ref transform): &(Ray3<Real>, T)) -> bool {
        ::collision::Sphere {
            center: *transform.position(),
            radius: self.radius,
        }.intersects(ray)
    }
}

impl<T> Continuous<(Ray3<Real>, T)> for Sphere
where
    T: Pose<Point3<Real>>,
{
    type Result = Point3<Real>;

    fn intersection(&self, &(ref ray, ref transform): &(Ray3<Real>, T)) -> Option<Point3<Real>> {
        ::collision::Sphere {
            center: *transform.position(),
            radius: self.radius,
        }.intersection(ray)
    }
}
