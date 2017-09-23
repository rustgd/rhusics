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
//! # use rhusics::collide::primitives::primitive2d::*;
//! use rhusics::collide::primitives::HasAABB;
//! let p : Primitive2 = Rectangle::new(10., 34.).into();
//! p.get_bound();
//! ```

pub use self::circle::Circle;
pub use self::polygon::ConvexPolygon;
pub use self::rectangle::Rectangle;

use cgmath::{Point2, Vector2};
use cgmath::prelude::*;
use collision::{Aabb2, Ray2};

use super::{HasAABB, SupportFunction, DiscreteTransformed, ContinuousTransformed};
use {Pose, Real};

mod circle;
mod rectangle;
mod polygon;

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

impl HasAABB for Primitive2 {
    type Aabb = Aabb2<Real>;

    fn get_bound(&self) -> Aabb2<Real> {
        match *self {
            Primitive2::Circle(ref circle) => circle.get_bound(),
            Primitive2::Rectangle(ref rectangle) => rectangle.get_bound(),
            Primitive2::ConvexPolygon(ref polygon) => polygon.get_bound(),
        }
    }
}

impl SupportFunction for Primitive2 {
    type Point = Point2<Real>;

    fn support_point<T>(&self, direction: &Vector2<Real>, transform: &T) -> Point2<Real>
    where
        T: Pose<Point2<Real>>,
    {
        match *self {
            Primitive2::Circle(ref circle) => circle.support_point(direction, transform),
            Primitive2::Rectangle(ref rectangle) => rectangle.support_point(direction, transform),
            Primitive2::ConvexPolygon(ref polygon) => polygon.support_point(direction, transform),
        }
    }
}

impl DiscreteTransformed<Ray2<Real>> for Primitive2 {
    type Point = Point2<Real>;

    fn intersects_transformed<T>(&self, ray: &Ray2<Real>, transform: &T) -> bool
    where
        T: Transform<Self::Point>,
    {
        match *self {
            Primitive2::Circle(ref circle) => circle.intersects_transformed(ray, transform),
            Primitive2::Rectangle(ref rectangle) => {
                rectangle.intersects_transformed(ray, transform)
            }
            Primitive2::ConvexPolygon(ref polygon) => {
                polygon.intersects_transformed(ray, transform)
            }
        }
    }
}

impl ContinuousTransformed<Ray2<Real>> for Primitive2 {
    type Point = Point2<Real>;
    type Result = Point2<Real>;

    fn intersection_transformed<T>(&self, ray: &Ray2<Real>, transform: &T) -> Option<Point2<Real>>
    where
        T: Transform<Point2<Real>>,
    {
        match *self {
            Primitive2::Circle(ref circle) => circle.intersection_transformed(ray, transform),
            Primitive2::Rectangle(ref rectangle) => {
                rectangle.intersection_transformed(ray, transform)
            }
            Primitive2::ConvexPolygon(ref polygon) => {
                polygon.intersection_transformed(ray, transform)
            }
        }
    }
}
