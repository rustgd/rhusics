//! Rectangle primitive

use cgmath::{Point2, Vector2};
use cgmath::num_traits::Float;
use cgmath::prelude::*;
use collision::{Aabb2, Ray2};
use collision::prelude::*;

use {Pose, Real};
use collide::primitives::{ContinuousTransformed, DiscreteTransformed, HasAABB, SupportFunction};

/// Rectangle primitive.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone)]
pub struct Rectangle {
    /// Dimensions of the rectangle
    pub dim: Vector2<Real>,
    half_dim: Vector2<Real>,
    corners: Vec<Point2<Real>>,
}

impl Rectangle {
    /// Create a new rectangle primitive from component dimensions
    pub fn new(dim_x: Real, dim_y: Real) -> Self {
        Self::new_impl(Vector2::new(dim_x, dim_y))
    }

    /// Create a new rectangle primitive from a vector of component dimensions
    pub fn new_impl(dim: Vector2<Real>) -> Self {
        Rectangle {
            dim,
            half_dim: dim / 2.,
            corners: Self::generate_corners(&dim),
        }
    }

    fn generate_corners(dimensions: &Vector2<Real>) -> Vec<Point2<Real>> {
        let two = 2.;
        vec![
            Point2::new(dimensions.x / two, dimensions.y / two),
            Point2::new(-dimensions.x / two, dimensions.y / two),
            Point2::new(-dimensions.x / two, -dimensions.y / two),
            Point2::new(dimensions.x / two, -dimensions.y / two),
        ]
    }
}

impl SupportFunction for Rectangle {
    type Point = Point2<Real>;

    fn support_point<T>(&self, direction: &Vector2<Real>, transform: &T) -> Point2<Real>
    where
        T: Pose<Point2<Real>>,
    {
        ::util::get_max_point(&self.corners, direction, transform)
    }
}

impl HasAABB for Rectangle {
    type Aabb = Aabb2<Real>;

    fn get_bound(&self) -> Aabb2<Real> {
        Aabb2::new(
            Point2::from_vec(-self.half_dim),
            Point2::from_vec(self.half_dim),
        )
    }
}

impl DiscreteTransformed<Ray2<Real>> for Rectangle {
    type Point = Point2<Real>;

    fn intersects_transformed<T>(&self, ray: &Ray2<Real>, transform: &T) -> bool
    where
        T: Transform<Point2<Real>>,
    {
        self.intersects(&ray.transform(transform.inverse_transform().unwrap()))
    }
}

impl ContinuousTransformed<Ray2<Real>> for Rectangle {
    type Point = Point2<Real>;
    type Result = Point2<Real>;

    fn intersection_transformed<T>(&self, ray: &Ray2<Real>, transform: &T) -> Option<Point2<Real>>
    where
        T: Transform<Point2<Real>>,
    {
        self.intersection(&ray.transform(transform.inverse_transform().unwrap()))
            .map(|p| transform.transform_point(p))
    }
}

impl Discrete<Ray2<Real>> for Rectangle {
    /// Ray must be in object space of the rectangle
    fn intersects(&self, ray: &Ray2<Real>) -> bool {
        let min = Point2::new(-self.half_dim.x, -self.half_dim.y);
        let max = Point2::new(self.half_dim.x, self.half_dim.y);

        let mut tmin = Real::neg_infinity();
        let mut tmax = Real::infinity();

        if ray.direction.x != 0. {
            let tx1 = (min.x - ray.origin.x) / ray.direction.x;
            let tx2 = (max.x - ray.origin.x) / ray.direction.x;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if ray.origin.x <= min.x || ray.origin.x >= max.x {
            return false;
        }

        if ray.direction.y != 0. {
            let ty1 = (min.y - ray.origin.y) / ray.direction.y;
            let ty2 = (max.y - ray.origin.y) / ray.direction.y;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if ray.origin.y <= min.y || ray.origin.y >= max.y {
            return false;
        }

        tmax >= tmin && (tmin >= 0. || tmax >= 0.)
    }
}

impl Continuous<Ray2<Real>> for Rectangle {
    type Result = Point2<Real>;

    /// Ray must be in object space of the rectangle
    fn intersection(&self, ray: &Ray2<Real>) -> Option<Point2<Real>> {
        let min = Point2::new(-self.half_dim.x, -self.half_dim.y);
        let max = Point2::new(self.half_dim.x, self.half_dim.y);
        let mut tmin = Real::neg_infinity();
        let mut tmax = Real::infinity();

        if ray.direction.x != 0. {
            let tx1 = (min.x - ray.origin.x) / ray.direction.x;
            let tx2 = (max.x - ray.origin.x) / ray.direction.x;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if ray.origin.x <= min.x || ray.origin.x >= max.x {
            return None;
        }

        if ray.direction.y != 0. {
            let ty1 = (min.y - ray.origin.y) / ray.direction.y;
            let ty2 = (max.y - ray.origin.y) / ray.direction.y;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if ray.origin.y <= min.y || ray.origin.y >= max.y {
            return None;
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
    use cgmath::{Point2, Vector2, Rad};

    use super::*;
    use collide2d::BodyPose2;

    #[test]
    fn test_rectangle_bound() {
        let r = Rectangle::new(10., 10.);
        assert_eq!(bound(-5., -5., 5., 5.), r.get_bound())
    }

    #[test]
    fn test_rectangle_ray_discrete() {
        let rectangle = Rectangle::new(10., 10.);
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        assert!(rectangle.intersects(&ray));
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(0., 1.));
        assert!(!rectangle.intersects(&ray));
    }

    #[test]
    fn test_rectangle_ray_discrete_transformed() {
        let rectangle = Rectangle::new(10., 10.);
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        let transform = BodyPose2::one();
        assert!(rectangle.intersects_transformed(&ray, &transform));
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        let transform = BodyPose2::new(Point2::new(0., 20.), Rotation2::from_angle(Rad(0.)));
        assert!(!rectangle.intersects_transformed(&ray, &transform));
    }

    #[test]
    fn test_rectangle_ray_continuous() {
        let rectangle = Rectangle::new(10., 10.);
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        assert_eq!(Some(Point2::new(5., 0.)), rectangle.intersection(&ray));
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(0., 1.));
        assert_eq!(None, rectangle.intersection(&ray));
    }

    #[test]
    fn test_rectangle_ray_continuous_transformed() {
        let rectangle = Rectangle::new(10., 10.);
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        let transform = BodyPose2::one();
        assert_eq!(
            Some(Point2::new(5., 0.)),
            rectangle.intersection_transformed(&ray, &transform)
        );
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        let transform = BodyPose2::new(Point2::new(0., 20.), Rotation2::from_angle(Rad(0.)));
        assert_eq!(None, rectangle.intersection_transformed(&ray, &transform));
        let transform = BodyPose2::new(Point2::new(0., 0.), Rotation2::from_angle(Rad(0.3)));
        let p = rectangle
            .intersection_transformed(&ray, &transform)
            .unwrap();
        assert_approx_eq!(5.233758, p.x);
        assert_approx_eq!(0., p.y);
    }

    // util
    fn bound(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Aabb2<Real> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
