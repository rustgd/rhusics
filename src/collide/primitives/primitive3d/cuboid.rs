use cgmath::{Point3, Vector3};
use cgmath::prelude::*;
use collision::{Aabb3, Ray3};
use collision::prelude::*;

use {Pose, Real};
use collide::primitives::{HasAABB, SupportFunction, ContinuousTransformed, DiscreteTransformed};

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

impl SupportFunction for Cuboid {
    type Point = Point3<Real>;

    fn support_point<T>(&self, direction: &Vector3<Real>, transform: &T) -> Point3<Real>
    where
        T: Pose<Point3<Real>>,
    {
        ::util::get_max_point(&self.corners, direction, transform)
    }
}

impl HasAABB for Cuboid {
    type Aabb = Aabb3<Real>;

    fn get_bound(&self) -> Aabb3<Real> {
        Aabb3::new(
            Point3::from_vec(-self.half_dim),
            Point3::from_vec(self.half_dim),
        )
    }
}

impl DiscreteTransformed<Ray3<Real>> for Cuboid {
    type Point = Point3<Real>;

    fn intersects_transformed<T>(&self, ray: &Ray3<Real>, transform: &T) -> bool
    where
        T: Transform<Point3<Real>>,
    {
        self.intersects(&ray.transform(transform.inverse_transform().unwrap()))
    }
}

impl Discrete<Ray3<Real>> for Cuboid {
    fn intersects(&self, ray: &Ray3<Real>) -> bool {
        let min = Point3::new(-self.half_dim.x, -self.half_dim.y, -self.half_dim.z);
        let max = Point3::new(self.half_dim.x, self.half_dim.y, self.half_dim.z);

        let inv_dir = Vector3::from_value(1.).div_element_wise(ray.direction);

        let mut t1 = (min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (max.x - ray.origin.x) * inv_dir.x;

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        for i in 1..3 {
            t1 = (min[i] - ray.origin[i]) * inv_dir[i];
            t2 = (max[i] - ray.origin[i]) * inv_dir[i];

            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        tmax >= tmin && (tmin >= 0. || tmax >= 0.)
    }
}

impl ContinuousTransformed<Ray3<Real>> for Cuboid {
    type Point = Point3<Real>;
    type Result = Point3<Real>;

    fn intersection_transformed<T>(&self, ray: &Ray3<Real>, transform: &T) -> Option<Point3<Real>>
    where
        T: Transform<Point3<Real>>,
    {
        self.intersection(&ray.transform(transform.inverse_transform().unwrap()))
            .map(|p| transform.transform_point(p))
    }
}

impl Continuous<Ray3<Real>> for Cuboid {
    type Result = Point3<Real>;

    fn intersection(&self, ray: &Ray3<Real>) -> Option<Point3<Real>> {
        let min = Point3::new(-self.half_dim.x, -self.half_dim.y, -self.half_dim.z);
        let max = Point3::new(self.half_dim.x, self.half_dim.y, self.half_dim.z);

        let inv_dir = Vector3::from_value(1.).div_element_wise(ray.direction);

        let mut t1 = (min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (max.x - ray.origin.x) * inv_dir.x;

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        for i in 1..3 {
            t1 = (min[i] - ray.origin[i]) * inv_dir[i];
            t2 = (max[i] - ray.origin[i]) * inv_dir[i];

            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        if (tmin < 0. && tmax < 0.) || tmax < tmin {
            None
        } else {
            let t = if tmin >= 0. { tmin } else { tmax };
            Some(ray.origin + ray.direction * t)
        }
    }
}


#[cfg(test)]
mod tests {

    use cgmath::{Point3, Vector3, Quaternion, Rad};
    use collision::Ray3;

    use super::*;
    use collide3d::BodyPose3;

    #[test]
    fn test_rectangle_bound() {
        let r = Cuboid::new(10., 10., 10.);
        assert_eq!(bound(-5., -5., -5., 5., 5., 5.), r.get_bound())
    }

    #[test]
    fn test_ray_discrete() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(cuboid.intersects(&ray));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(!cuboid.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        let transform = BodyPose3::new(Point3::new(0., 1., 0.), Quaternion::one());
        assert!(cuboid.intersects_transformed(&ray, &transform));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(!cuboid.intersects_transformed(&ray, &transform));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        let transform = BodyPose3::new(Point3::new(0., 1., 0.), Quaternion::from_angle_z(Rad(0.3)));
        assert!(cuboid.intersects_transformed(&ray, &transform));
    }

    #[test]
    fn test_ray_continuous() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(Some(Point3::new(5., 0., 0.)), cuboid.intersection(&ray));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(None, cuboid.intersection(&ray));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        let transform = BodyPose3::new(Point3::new(0., 1., 0.), Quaternion::one());
        assert_eq!(
            Some(Point3::new(5., 0., 0.)),
            cuboid.intersection_transformed(&ray, &transform)
        );
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(None, cuboid.intersection_transformed(&ray, &transform));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        let transform = BodyPose3::new(Point3::new(0., 0., 0.), Quaternion::from_angle_z(Rad(0.3)));
        let p = cuboid.intersection_transformed(&ray, &transform).unwrap();
        assert_approx_eq!(5.233758, p.x);
        assert_approx_eq!(0., p.y);
        assert_approx_eq!(0., p.z);
    }

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
