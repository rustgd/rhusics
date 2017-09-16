//! Collision primitives for 2D.
//!
//! These are the building blocks of all collision detection for 2D.
//! The enum [`Primitive2`](enum.Primitive2.html) is the main type, that implements the
//! [`Primitive`](../trait.Primitive.html) trait.
//!
//! All primitive types have `Into` implementations to turn them into enum variants for
//! [`Primitive2`](enum.Primitive2.html).
//!
//! # Example
//!
//! ```
//! # use rhusics::collide::primitive2d::*;
//! use rhusics::collide::Primitive;
//! let p : Primitive2 = Rectangle::new(10., 34.).into();
//! p.get_bound();
//! ```

use cgmath::{Vector2, Point2};
use cgmath::prelude::*;
use collision::Aabb2;

use super::Primitive;
use {Pose, Real};

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

/// Convex polygon primitive.
///
/// Can contain any number of vertices, but a high number of vertices will
/// affect performance of course. Vertices need to be in CCW order.
#[derive(Debug, Clone)]
pub struct ConvexPolygon {
    /// Vertices of the convex polygon.
    pub vertices: Vec<Point2<Real>>,
}

impl ConvexPolygon {
    /// Create a new convex polygon from the given vertices. Vertices need to be in CCW order.
    pub fn new(vertices: Vec<Point2<Real>>) -> Self {
        Self { vertices }
    }
}

/// Base enum for all 2D primitives
#[derive(Debug, Clone)]
pub enum Primitive2 {
    /// Circle variant
    Circle(Circle),

    /// Rectangle variant
    Rectangle(Rectangle),

    /// Convex polygon variant
    ConvexPolygon(ConvexPolygon),
}

impl Into<Primitive2> for Circle {
    fn into(self) -> Primitive2 {
        Primitive2::Circle(self)
    }
}

impl Into<Primitive2> for Rectangle {
    fn into(self) -> Primitive2 {
        Primitive2::Rectangle(self)
    }
}

impl Into<Primitive2> for ConvexPolygon {
    fn into(self) -> Primitive2 {
        Primitive2::ConvexPolygon(self)
    }
}

impl Primitive for Primitive2 {
    type Vector = Vector2<Real>;
    type Point = Point2<Real>;
    type Aabb = Aabb2<Real>;

    fn get_bound(&self) -> Aabb2<Real> {
        match *self {
            Primitive2::Circle(ref circle) => {
                Aabb2::new(
                    Point2::from_value(-circle.radius),
                    Point2::from_value(circle.radius),
                )
            }
            Primitive2::Rectangle(ref rectangle) => {
                Aabb2::new(
                    Point2::from_vec(-rectangle.half_dim),
                    Point2::from_vec(rectangle.half_dim),
                )
            }

            Primitive2::ConvexPolygon(ref polygon) => ::util::get_bound(&polygon.vertices),
        }
    }

    fn get_far_point<T>(&self, direction: &Vector2<Real>, transform: &T) -> Point2<Real>
    where
        T: Pose<Point2<Real>>,
    {
        match *self {
            Primitive2::Circle(ref circle) => {
                transform.position() + direction.normalize_to(circle.radius)
            }

            Primitive2::Rectangle(Rectangle { ref corners, .. }) => {
                ::util::get_max_point(corners, direction, transform)
            }

            Primitive2::ConvexPolygon(ConvexPolygon { ref vertices, .. }) => {
                if vertices.len() < 10 {
                    ::util::get_max_point(vertices, direction, transform)
                } else {
                    get_max_point(vertices, direction, transform)
                }
            }
        }
    }
}

fn get_max_point<P, T>(vertices: &Vec<P>, direction: &P::Diff, transform: &T) -> P
where
    P: EuclideanSpace<Scalar = Real>,
    T: Pose<P>,
{
    let direction = transform.inverse_rotation().rotate_vector(*direction);

    // figure out where to start, if the direction is negative for the first vertex,
    // go halfway around the polygon
    let mut start_index: i32 = 0;
    let mut max_dot = vertices[0].dot(direction);
    if max_dot < P::Scalar::zero() {
        start_index = vertices.len() as i32 / 2;
        max_dot = dot_index(vertices, start_index, &direction);
    }

    let left_dot = dot_index(vertices, start_index - 1, &direction);
    let right_dot = dot_index(vertices, start_index + 1, &direction);

    // check if start is highest
    let p = if start_index == 0 && max_dot > left_dot && max_dot > right_dot {
        vertices[0]
    } else {
        // figure out iteration direction
        let mut add: i32 = 1;
        let mut previous_dot = left_dot;
        if left_dot > max_dot && left_dot > right_dot {
            add = -1;
            previous_dot = right_dot;
        }

        // iterate
        let mut index = start_index + add;
        let mut current_dot = max_dot;
        if index == vertices.len() as i32 {
            index = 0;
        }
        if index == -1 {
            index = vertices.len() as i32 - 1;
        }
        while index != start_index {
            let next_dot = dot_index(vertices, index + add, &direction);
            if current_dot > previous_dot && current_dot > next_dot {
                break;
            }
            previous_dot = current_dot;
            current_dot = next_dot;
            index += add;
            if index == vertices.len() as i32 {
                index = 0;
            }
            if index == -1 {
                index = vertices.len() as i32 - 1;
            }
        }
        vertices[index as usize]
    };

    *transform.position() + transform.rotation().rotate_point(p).to_vec()
}

#[inline]
fn dot_index<P>(vertices: &Vec<P>, index: i32, direction: &P::Diff) -> Real
where
    P: EuclideanSpace<Scalar = Real>,
{
    let index_u = index as usize;
    if index_u == vertices.len() {
        vertices[0].dot(*direction)
    } else if index == -1 {
        vertices[vertices.len() - 1].dot(*direction)
    } else {
        vertices[index_u].dot(*direction)
    }
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::{Point2, Vector2, Rotation2, Rad, Basis2};

    use super::*;
    use BodyPose;
    use collide2d::BodyPose2;

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
        let circle: Primitive2 = Circle::new(10.).into();
        let direction = Vector2::new(1., 0.);
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 10.), Rotation2::from_angle(Rad(0.)));
        let point = circle.get_far_point(&direction, &transform);
        assert_eq!(Point2::new(10., 10.), point);
    }

    #[test]
    fn test_circle_bound() {
        let circle: Primitive2 = Circle::new(10.).into();
        assert_eq!(bound(-10., -10., 10., 10.), circle.get_bound())
    }

    fn test_circle(dx: Real, dy: Real, px: Real, py: Real, rot: Real) {
        let circle: Primitive2 = Circle::new(10.).into();
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
        let r: Primitive2 = Rectangle::new(10., 10.).into();
        assert_eq!(bound(-5., -5., 5., 5.), r.get_bound())
    }

    // convex polygon
    // not testing bound as ::util::get_bound is fairly well tested
    // not testing far point as ::util::get_max_point is rigorously tested

    // util
    fn bound(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Aabb2<Real> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }

    #[test]
    fn test_max_point() {
        let vertices = vec![
            Point2::new(5., 5.),
            Point2::new(4., 6.),
            Point2::new(3., 7.),
            Point2::new(2., 6.),
            Point2::new(1., 6.),
            Point2::new(0., 5.),
            Point2::new(-1., 4.),
            Point2::new(-3., 3.),
            Point2::new(-6., 1.),
            Point2::new(-5., 0.),
            Point2::new(-4., -1.),
            Point2::new(-2., -3.),
            Point2::new(0., -7.),
            Point2::new(1., -8.),
            Point2::new(2., -5.),
            Point2::new(3., 0.),
            Point2::new(4., 3.),
        ];
        let transform = BodyPose2::one();
        let point = get_max_point(&vertices, &Vector2::new(-1., 0.), &transform);
        assert_eq!(Point2::new(-5., 0.), point);

        let point = get_max_point(&vertices, &Vector2::new(0., -1.), &transform);
        assert_eq!(Point2::new(1., -8.), point);

        let point = get_max_point(&vertices, &Vector2::new(0., 1.), &transform);
        assert_eq!(Point2::new(3., 7.), point);

        let point = get_max_point(&vertices, &Vector2::new(1., 0.), &transform);
        assert_eq!(Point2::new(5., 5.), point);
    }
}
