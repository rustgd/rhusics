use cgmath::prelude::*;
use cgmath::{BaseFloat, Vector2, Point2, Decomposed};
use collision::Aabb2;
use super::Primitive;

#[derive(Debug, Clone)]
pub struct Circle<S> {
    pub radius: S,
}

impl<S> Circle<S> {
    pub fn new(radius: S) -> Self {
        Self { radius }
    }
}

#[derive(Debug, Clone)]
pub struct Rectangle<S> {
    pub dim: Vector2<S>,
    half_dim: Vector2<S>,
    corners: Vec<Point2<S>>,
}

impl<S: BaseFloat> Rectangle<S> {
    pub fn new(dim_x: S, dim_y: S) -> Self {
        Self::new_impl(Vector2::new(dim_x, dim_y))
    }

    pub fn new_impl(dim: Vector2<S>) -> Self {
        let two = S::one() + S::one();
        Rectangle {
            dim,
            half_dim: dim / two,
            corners: Self::generate_corners(&dim),
        }
    }

    pub fn generate_corners(dimensions: &Vector2<S>) -> Vec<Point2<S>> {
        let two = S::one() + S::one();
        vec![
            Point2::new(dimensions.x / two, dimensions.y / two),
            Point2::new(-dimensions.x / two, dimensions.y / two),
            Point2::new(-dimensions.x / two, -dimensions.y / two),
            Point2::new(dimensions.x / two, -dimensions.y / two),
        ]
    }
}

#[derive(Debug, Clone)]
pub struct ConvexPolygon<S> {
    pub vertices: Vec<Point2<S>>,
}

#[derive(Debug, Clone)]
pub enum Primitive2D<S> {
    Circle(Circle<S>),
    Rectangle(Rectangle<S>),
    ConvexPolygon(ConvexPolygon<S>),
}

impl<S> Into<Primitive2D<S>> for Circle<S> {
    fn into(self) -> Primitive2D<S> {
        Primitive2D::Circle(self)
    }
}

impl<S> Into<Primitive2D<S>> for Rectangle<S> {
    fn into(self) -> Primitive2D<S> {
        Primitive2D::Rectangle(self)
    }
}

impl<S> Into<Primitive2D<S>> for ConvexPolygon<S> {
    fn into(self) -> Primitive2D<S> {
        Primitive2D::ConvexPolygon(self)
    }
}

impl<S> Primitive<Aabb2<S>> for Primitive2D<S>
where
    S: BaseFloat + Send + Sync,
{
    fn get_bound(&self) -> Aabb2<S> {
        match *self {
            Primitive2D::Circle(ref circle) => {
                Aabb2::new(
                    Point2::new(-circle.radius, -circle.radius),
                    Point2::new(circle.radius, circle.radius),
                )
            }
            Primitive2D::Rectangle(ref rectangle) => {
                Aabb2::new(
                    Point2::new(-rectangle.half_dim.x, -rectangle.half_dim.y),
                    Point2::new(rectangle.half_dim.x, rectangle.half_dim.y),
                )
            }
            Primitive2D::ConvexPolygon(ref polygon) => ::util::get_bound(&polygon.vertices),
        }
    }

    fn get_far_point<R>(
        &self,
        direction: &Vector2<S>,
        transform: &Decomposed<Vector2<S>, R>,
    ) -> Point2<S>
    where
        R: Rotation<Point2<S>>,
    {
        match *self {
            Primitive2D::Circle(ref circle) => {
                let direction = transform.rot.invert().rotate_vector(*direction);
                Point2::from_vec(transform.disp + direction.normalize_to(circle.radius))
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
    use super::*;
    use cgmath::{Decomposed, Point2, Vector2, Rotation2, Rad, Basis2};

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
        test_circle(1., 0., 7.0710683, 7.0710683, -std::f32::consts::PI / 4.);
    }

    #[test]
    fn test_circle_far_4() {
        let circle: Primitive2D<f32> = Circle::new(10.).into();
        let direction = Vector2::new(1., 0.);
        let transform: Decomposed<Vector2<f32>, Basis2<f32>> = Decomposed {
            disp: Vector2::new(0., 10.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let point = circle.get_far_point(&direction, &transform);
        assert_eq!(Point2::new(10., 10.), point);
    }

    #[test]
    fn test_circle_bound() {
        let circle: Primitive2D<f32> = Circle::new(10.).into();
        assert_eq!(bound(-10., -10., 10., 10.), circle.get_bound())
    }

    fn test_circle(dx: f32, dy: f32, px: f32, py: f32, rot: f32) {
        let circle: Primitive2D<f32> = Circle::new(10.).into();
        let direction = Vector2::new(dx, dy);
        let transform: Decomposed<Vector2<f32>, Basis2<f32>> = Decomposed {
            disp: Vector2::new(0., 0.),
            rot: Rotation2::from_angle(Rad(rot)),
            scale: 1.,
        };
        let point = circle.get_far_point(&direction, &transform);
        assert_eq!(Point2::new(px, py), point);
    }

    // rectangle
    // not testing far point as ::util::get_max_point is rigorously tested
    #[test]
    fn test_rectangle_bound() {
        let r: Primitive2D<f32> = Rectangle::new(10., 10.).into();
        assert_eq!(bound(-5., -5., 5., 5.), r.get_bound())
    }

    // convex polygon
    // not testing bound as ::util::get_bound is fairly well tested
    // not testing far point as ::util::get_max_point is rigorously tested

    // util
    fn bound(min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Aabb2<f32> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
