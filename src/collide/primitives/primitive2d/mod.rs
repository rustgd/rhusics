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
//! use rhusics::collide::Primitive;
//! let p : Primitive2 = Rectangle::new(10., 34.).into();
//! p.get_bound();
//! ```

pub use self::circle::Circle;
pub use self::polygon::ConvexPolygon;
pub use self::rectangle::Rectangle;

use cgmath::{Point2, Vector2};
use collision::Aabb2;

use super::{Primitive, HasAABB, SupportFunction};
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

impl Primitive for Primitive2 {
    type Vector = Vector2<Real>;
    type Point = Point2<Real>;
    type Aabb = Aabb2<Real>;

    fn get_bound(&self) -> Aabb2<Real> {
        match *self {
            Primitive2::Circle(ref circle) => circle.get_bound(),
            Primitive2::Rectangle(ref rectangle) => rectangle.get_bound(),
            Primitive2::ConvexPolygon(ref polygon) => polygon.get_bound(),
        }
    }

    fn get_far_point<T>(&self, direction: &Vector2<Real>, transform: &T) -> Point2<Real>
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
