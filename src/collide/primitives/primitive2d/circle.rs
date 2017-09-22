//! Circle primitive

use cgmath::{Point2, Vector2};
use cgmath::prelude::*;
use collision::{Aabb2, Ray2};
use collision::prelude::*;

use {Pose, Real};
use collide::primitives::{ContinuousTransformed, DiscreteTransformed, HasAABB, SupportFunction};

/// Circle primitive
#[derive(Debug, Clone)]
pub struct Circle {
    /// Radius of the circle
    pub radius: Real,
}

impl Circle {
    /// Create a new circle primitive
    pub fn new(radius: Real) -> Self {
        Self { radius }
    }
}

impl SupportFunction for Circle {
    type Point = Point2<Real>;

    fn support_point<T>(&self, direction: &Vector2<Real>, transform: &T) -> Point2<Real>
    where
        T: Pose<Point2<Real>>,
    {
        transform.position() + direction.normalize_to(self.radius)
    }
}

impl HasAABB for Circle {
    type Aabb = Aabb2<Real>;

    fn get_bound(&self) -> Aabb2<Real> {
        Aabb2::new(
            Point2::from_value(-self.radius),
            Point2::from_value(self.radius),
        )
    }
}

impl DiscreteTransformed<Ray2<Real>> for Circle {
    type Point = Point2<Real>;

    fn intersects_transformed<T>(&self, ray: &Ray2<Real>, transform: &T) -> bool
    where
        T: Transform<Point2<Real>>,
    {
        self.intersects(&(*ray, transform.transform_point(Point2::from_value(0.))))
    }
}

impl Discrete<(Ray2<Real>, Point2<Real>)> for Circle {
    fn intersects(&self, &(ref r, ref center): &(Ray2<Real>, Point2<Real>)) -> bool {
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

impl ContinuousTransformed<Ray2<Real>> for Circle {
    type Point = Point2<Real>;
    type Result = Point2<Real>;

    #[inline]
    fn intersection_transformed<T>(&self, ray: &Ray2<Real>, transform: &T) -> Option<Point2<Real>>
    where
        T: Transform<Point2<Real>>,
    {
        self.intersection(&(*ray, transform.transform_point(Point2::from_value(0.))))
    }
}

impl Continuous<(Ray2<Real>, Point2<Real>)> for Circle {
    type Result = Point2<Real>;

    fn intersection(
        &self,
        &(ref r, ref center): &(Ray2<Real>, Point2<Real>),
    ) -> Option<Point2<Real>> {
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

    use cgmath::{Basis2, Point2, Rad, Rotation2, Vector2};
    use collision::Ray2;
    use collision::prelude::*;

    use super::*;
    use BodyPose;

    // circle
    #[test]
    fn test_circle_far_1() {
        test_circle(1., 0., 10., 0., 0.);
    }

    #[test]
    fn test_circle_far_2() {
        test_circle(1., 1., 7.0710677, 7.0710677, 0.);
    }

    #[test]
    fn test_circle_far_3() {
        test_circle(1., 0., 10., 0., -std::f64::consts::PI as Real / 4.);
    }

    #[test]
    fn test_circle_far_4() {
        let circle = Circle::new(10.);
        let direction = Vector2::new(1., 0.);
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 10.), Rotation2::from_angle(Rad(0.)));
        let point = circle.support_point(&direction, &transform);
        assert_eq!(Point2::new(10., 10.), point);
    }

    #[test]
    fn test_circle_bound() {
        let circle = Circle::new(10.);
        assert_eq!(bound(-10., -10., 10., 10.), circle.get_bound())
    }

    #[test]
    fn test_circle_ray_discrete() {
        let circle = Circle::new(10.);
        let ray = Ray2::new(Point2::new(25., 0.), Vector2::new(-1., 0.));
        let center = Point2::new(0., 0.);
        assert!(circle.intersects(&(ray, center)));
        let center = Point2::new(0., 11.);
        assert!(!circle.intersects(&(ray, center)));
    }

    #[test]
    fn test_circle_ray_discrete_transformed() {
        let circle = Circle::new(10.);
        let ray = Ray2::new(Point2::new(25., 0.), Vector2::new(-1., 0.));
        let transform: BodyPose<Point2<Real>, Basis2<Real>> = BodyPose::one();
        assert!(circle.intersects_transformed(&ray, &transform));
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 11.), Rotation2::from_angle(Rad(0.)));
        assert!(!circle.intersects_transformed(&ray, &transform));
    }

    #[test]
    fn test_circle_ray_continuous() {
        let circle = Circle::new(10.);
        let ray = Ray2::new(Point2::new(25., 0.), Vector2::new(-1., 0.));
        let center = Point2::new(0., 0.);
        assert_eq!(
            Some(Point2::new(10., 0.)),
            circle.intersection(&(ray, center))
        );
        let center = Point2::new(0., 11.);
        assert_eq!(None, circle.intersection(&(ray, center)));
    }

    #[test]
    fn test_circle_ray_continuous_transformed() {
        let circle = Circle::new(10.);
        let ray = Ray2::new(Point2::new(25., 0.), Vector2::new(-1., 0.));
        let transform: BodyPose<Point2<Real>, Basis2<Real>> = BodyPose::one();
        assert_eq!(
            Some(Point2::new(10., 0.)),
            circle.intersection_transformed(&ray, &transform)
        );
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 11.), Rotation2::from_angle(Rad(0.)));
        assert_eq!(None, circle.intersection_transformed(&ray, &transform));
    }

    fn test_circle(dx: Real, dy: Real, px: Real, py: Real, rot: Real) {
        let circle = Circle::new(10.);
        let direction = Vector2::new(dx, dy);
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 0.), Rotation2::from_angle(Rad(rot)));
        let point = circle.support_point(&direction, &transform);
        assert_approx_eq!(px, point.x);
        assert_approx_eq!(py, point.y);
    }

    // util
    fn bound(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Aabb2<Real> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
