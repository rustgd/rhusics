use cgmath::{Vector2, Point2};
use cgmath::prelude::*;
use collision::Aabb2;

use super::Primitive;
use {Pose, Real};

#[derive(Debug, Clone)]
pub struct Circle {
    pub radius: Real,
}

impl Circle {
    pub fn new(radius: Real) -> Self {
        Self { radius }
    }
}

#[derive(Debug, Clone)]
pub struct Rectangle {
    pub dim: Vector2<Real>,
    half_dim: Vector2<Real>,
    corners: Vec<Point2<Real>>,
}

impl Rectangle {
    pub fn new(dim_x: Real, dim_y: Real) -> Self {
        Self::new_impl(Vector2::new(dim_x, dim_y))
    }

    pub fn new_impl(dim: Vector2<Real>) -> Self {
        Rectangle {
            dim,
            half_dim: dim / 2.,
            corners: Self::generate_corners(&dim),
        }
    }

    pub fn generate_corners(dimensions: &Vector2<Real>) -> Vec<Point2<Real>> {
        let two = 2.;
        vec![
            Point2::new(dimensions.x / two, dimensions.y / two),
            Point2::new(-dimensions.x / two, dimensions.y / two),
            Point2::new(-dimensions.x / two, -dimensions.y / two),
            Point2::new(dimensions.x / two, -dimensions.y / two),
        ]
    }
}

#[derive(Debug, Clone)]
pub struct ConvexPolygon {
    pub vertices: Vec<Point2<Real>>,
}

#[derive(Debug, Clone)]
pub enum Primitive2D {
    Circle(Circle),
    Rectangle(Rectangle),
    ConvexPolygon(ConvexPolygon),
}

impl Into<Primitive2D> for Circle {
    fn into(self) -> Primitive2D {
        Primitive2D::Circle(self)
    }
}

impl Into<Primitive2D> for Rectangle {
    fn into(self) -> Primitive2D {
        Primitive2D::Rectangle(self)
    }
}

impl Into<Primitive2D> for ConvexPolygon {
    fn into(self) -> Primitive2D {
        Primitive2D::ConvexPolygon(self)
    }
}

impl Primitive<Aabb2<Real>> for Primitive2D {
    fn get_bound(&self) -> Aabb2<Real> {
        match *self {
            Primitive2D::Circle(ref circle) => {
                Aabb2::new(
                    Point2::from_value(-circle.radius),
                    Point2::from_value(circle.radius),
                )
            }
            Primitive2D::Rectangle(ref rectangle) => {
                Aabb2::new(
                    Point2::from_vec(-rectangle.half_dim),
                    Point2::from_vec(rectangle.half_dim),
                )
            }
            Primitive2D::ConvexPolygon(ref polygon) => ::util::get_bound(&polygon.vertices),
        }
    }

    fn get_far_point<P>(&self, direction: &Vector2<Real>, transform: &P) -> Point2<Real>
    where
        P: Pose<Point2<Real>>,
    {
        match *self {
            Primitive2D::Circle(ref circle) => {
                let direction = transform.inverse_rotation().rotate_vector(*direction);
                transform.position() + direction.normalize_to(circle.radius)
            }
            Primitive2D::Rectangle(Rectangle { ref corners, .. }) => {
                ::util::get_max_point(corners, direction, transform)
            }
            Primitive2D::ConvexPolygon(ConvexPolygon { ref vertices, .. }) => {
                ::util::get_max_point(vertices, direction, transform)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::{Point2, Vector2, Rotation2, Rad, Basis2};

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
        test_circle(
            1.,
            0.,
            7.0710683,
            7.0710683,
            -std::f64::consts::PI as Real / 4.,
        );
    }

    #[test]
    fn test_circle_far_4() {
        let circle: Primitive2D = Circle::new(10.).into();
        let direction = Vector2::new(1., 0.);
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 10.), Rotation2::from_angle(Rad(0.)));
        let point = circle.get_far_point(&direction, &transform);
        assert_eq!(Point2::new(10., 10.), point);
    }

    #[test]
    fn test_circle_bound() {
        let circle: Primitive2D = Circle::new(10.).into();
        assert_eq!(bound(-10., -10., 10., 10.), circle.get_bound())
    }

    fn test_circle(dx: Real, dy: Real, px: Real, py: Real, rot: Real) {
        let circle: Primitive2D = Circle::new(10.).into();
        let direction = Vector2::new(dx, dy);
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 0.), Rotation2::from_angle(Rad(rot)));
        let point = circle.get_far_point(&direction, &transform);
        assert_approx_eq!(px, point.x);
        assert_approx_eq!(py, point.y);
    }

    // rectangle
    // not testing far point as ::util::get_max_point is rigorously tested
    #[test]
    fn test_rectangle_bound() {
        let r: Primitive2D = Rectangle::new(10., 10.).into();
        assert_eq!(bound(-5., -5., 5., 5.), r.get_bound())
    }

    // convex polygon
    // not testing bound as ::util::get_bound is fairly well tested
    // not testing far point as ::util::get_max_point is rigorously tested

    // util
    fn bound(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Aabb2<Real> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
