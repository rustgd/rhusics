use cgmath::{Point3, Vector3};
use cgmath::prelude::*;
use collision::{Aabb3, Ray3};
use collision::prelude::*;

use {Pose, Real};
use collide::primitives::{HasAABB, SupportFunction, ContinuousTransformed, DiscreteTransformed};

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
}

impl SupportFunction for Sphere {
    type Point = Point3<Real>;

    fn support_point<T>(&self, direction: &Vector3<Real>, transform: &T) -> Point3<Real>
    where
        T: Pose<Point3<Real>>,
    {
        transform.position() + direction.normalize_to(self.radius)
    }
}

impl HasAABB for Sphere {
    type Aabb = Aabb3<Real>;

    fn get_bound(&self) -> Aabb3<Real> {
        Aabb3::new(
            Point3::from_value(-self.radius),
            Point3::from_value(self.radius),
        )
    }
}

impl DiscreteTransformed<Ray3<Real>> for Sphere {
    type Point = Point3<Real>;

    fn intersects_transformed<T>(&self, ray: &Ray3<Real>, transform: &T) -> bool
    where
        T: Transform<Point3<Real>>,
    {
        self.intersects(&(*ray, transform.transform_point(Point3::from_value(0.))))
    }
}

impl Discrete<(Ray3<Real>, Point3<Real>)> for Sphere {
    fn intersects(&self, &(ref r, ref center): &(Ray3<Real>, Point3<Real>)) -> bool {
        let s = self;
        let l = center - r.origin;
        let tca = l.dot(r.direction);
        if tca < 0. {
            return false;
        }
        let d2 = l.dot(l) - tca * tca;
        if d2 > s.radius * s.radius {
            return false;
        }
        return true;
    }
}

impl ContinuousTransformed<Ray3<Real>> for Sphere {
    type Result = Point3<Real>;
    type Point = Point3<Real>;

    fn intersection_transformed<T>(&self, ray: &Ray3<Real>, transform: &T) -> Option<Point3<Real>>
    where
        T: Transform<Point3<Real>>,
    {
        self.intersection(&(*ray, transform.transform_point(Point3::from_value(0.))))
    }
}

impl Continuous<(Ray3<Real>, Point3<Real>)> for Sphere {
    type Result = Point3<Real>;

    fn intersection(
        &self,
        &(ref r, ref center): &(Ray3<Real>, Point3<Real>),
    ) -> Option<Point3<Real>> {
        let s = self;

        let l = center - r.origin;
        let tca = l.dot(r.direction);
        if tca < 0. {
            return None;
        }
        let d2 = l.dot(l) - tca * tca;
        if d2 > s.radius * s.radius {
            return None;
        }
        let thc = (s.radius * s.radius - d2).sqrt();
        Some(r.origin + r.direction * (tca - thc))
    }
}


#[cfg(test)]
mod tests {
    use std;

    use cgmath::{Point3, Quaternion, Rad, Rotation3, Vector3};

    use super::*;
    use BodyPose;
    use collide3d::BodyPose3;

    // sphere
    #[test]
    fn test_sphere_support_1() {
        test_sphere_support(1., 0., 0., 10., 0., 0., 0.);
    }

    #[test]
    fn test_sphere_support_2() {
        test_sphere_support(
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
    fn test_sphere_support_3() {
        test_sphere_support(1., 0., 0., 10., 0., 0., -std::f64::consts::PI as Real / 4.);
    }

    #[test]
    fn test_sphere_support_4() {
        let sphere = Sphere::new(10.);
        let direction = Vector3::new(1., 0., 0.);
        let transform: BodyPose<Point3<Real>, Quaternion<Real>> =
            BodyPose::new(Point3::new(0., 10., 0.), Quaternion::from_angle_z(Rad(0.)));
        let point = sphere.support_point(&direction, &transform);
        assert_eq!(Point3::new(10., 10., 0.), point);
    }

    #[test]
    fn test_sphere_bound() {
        let sphere = Sphere::new(10.);
        assert_eq!(bound(-10., -10., -10., 10., 10., 10.), sphere.get_bound())
    }

    #[test]
    fn test_ray_discrete() {
        let sphere = Sphere::new(10.);
        let center = Point3::from_value(0.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(sphere.intersects(&(ray, center)));
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(!sphere.intersects(&(ray, center)));
        let center = Point3::new(0., 15., 0.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(!sphere.intersects(&(ray, center)));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let sphere = Sphere::new(10.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        let transform = BodyPose3::new(Point3::new(0., 0., 0.), Quaternion::one());
        assert!(sphere.intersects_transformed(&ray, &transform));
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(!sphere.intersects_transformed(&ray, &transform));
        let transform = BodyPose3::new(Point3::new(0., 15., 0.), Quaternion::one());
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(!sphere.intersects_transformed(&ray, &transform));
    }

    #[test]
    fn test_ray_continuous() {
        let sphere = Sphere::new(10.);
        let center = Point3::from_value(0.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(
            Some(Point3::new(10., 0., 0.)),
            sphere.intersection(&(ray, center))
        );
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(None, sphere.intersection(&(ray, center)));
        let center = Point3::new(0., 15., 0.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(None, sphere.intersection(&(ray, center)));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let sphere = Sphere::new(10.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        let transform = BodyPose3::new(Point3::new(0., 0., 0.), Quaternion::one());
        assert_eq!(
            Some(Point3::new(10., 0., 0.)),
            sphere.intersection_transformed(&ray, &transform)
        );
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(None, sphere.intersection_transformed(&ray, &transform));
        let transform = BodyPose3::new(Point3::new(0., 15., 0.), Quaternion::one());
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(None, sphere.intersection_transformed(&ray, &transform));
    }

    fn test_sphere_support(dx: Real, dy: Real, dz: Real, px: Real, py: Real, pz: Real, rot: Real) {
        let sphere = Sphere::new(10.);
        let direction = Vector3::new(dx, dy, dz);
        let transform: BodyPose<Point3<Real>, Quaternion<Real>> =
            BodyPose::new(Point3::new(0., 0., 0.), Quaternion::from_angle_z(Rad(rot)));
        let point = sphere.support_point(&direction, &transform);
        assert_approx_eq!(px, point.x);
        assert_approx_eq!(py, point.y);
        assert_approx_eq!(pz, point.z);
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
