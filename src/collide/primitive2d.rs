use cgmath::prelude::*;
use cgmath::{BaseFloat, Vector2, Point2, Basis2, Decomposed};
use collision::Aabb2;
use super::Primitive;

#[derive(Debug)]
pub struct Circle<S: BaseFloat> {
    pub radius: S,
}

impl<S: BaseFloat> Circle<S> {
    pub fn new(radius: S) -> Self {
        Self { radius }
    }
}

#[derive(Debug)]
pub struct Rectangle<S: BaseFloat> {
    pub dim: Vector2<S>,
    half_dim: Vector2<S>,
    corners: Vec<Vector2<S>>,
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

    pub fn generate_corners(dimensions: &Vector2<S>) -> Vec<Vector2<S>> {
        let two = S::one() + S::one();
        vec![
            Vector2::new(dimensions.x / two, dimensions.y / two),
            Vector2::new(-dimensions.x / two, dimensions.y / two),
            Vector2::new(-dimensions.x / two, -dimensions.y / two),
            Vector2::new(dimensions.x / two, -dimensions.y / two),
        ]
    }
}

#[derive(Debug)]
pub struct ConvexPolygon<S: BaseFloat> {
    pub vertices: Vec<Vector2<S>>,
}

impl<S> Primitive<S, Vector2<S>, Point2<S>, Basis2<S>, Aabb2<S>> for Circle<S>
where
    S: BaseFloat
        + Send
        + Sync,
{
    fn get_far_point(
        &self,
        direction: &Vector2<S>,
        transform: &Decomposed<Vector2<S>, Basis2<S>>,
    ) -> Point2<S> {
        let direction = transform.rot.invert().rotate_vector(*direction);
        Point2::from_vec(transform.disp + direction.normalize_to(self.radius))
    }

    fn get_bound(&self) -> Aabb2<S> {
        Aabb2::new(
            Point2::new(-self.radius, -self.radius),
            Point2::new(self.radius, self.radius),
        )
    }
}

impl<S> Primitive<S, Vector2<S>, Point2<S>, Basis2<S>, Aabb2<S>> for Rectangle<S>
where
    S: BaseFloat
        + Send
        + Sync,
{
    fn get_far_point(
        &self,
        direction: &Vector2<S>,
        transform: &Decomposed<Vector2<S>, Basis2<S>>,
    ) -> Point2<S> {
        ::util::get_max_point(&self.corners, direction, transform)
    }

    fn get_bound(&self) -> Aabb2<S> {
        Aabb2::new(
            Point2::new(-self.half_dim.x, -self.half_dim.y),
            Point2::new(self.half_dim.x, self.half_dim.y),
        )
    }
}

impl<S> Primitive<S, Vector2<S>, Point2<S>, Basis2<S>, Aabb2<S>> for ConvexPolygon<S>
where
    S: BaseFloat
        + Send
        + Sync,
{
    fn get_far_point(
        &self,
        direction: &Vector2<S>,
        transform: &Decomposed<Vector2<S>, Basis2<S>>,
    ) -> Point2<S> {
        ::util::get_max_point(&self.vertices, direction, transform)
    }

    fn get_bound(&self) -> Aabb2<S> {
        ::util::get_bound(&self.vertices)
    }
}

#[cfg(test)]
mod tests {
    use std;
    use super::*;
    use cgmath::{Decomposed, Point2, Vector2, Rotation2, Rad};

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
        let circle = Circle::new(10.);
        let direction = Vector2::new(1., 0.);
        let transform = Decomposed {
            disp: Vector2::new(0., 10.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let point = circle.get_far_point(&direction, &transform);
        assert_eq!(Point2::new(10., 10.), point);
    }

    #[test]
    fn test_circle_bound() {
        let circle = Circle::new(10.);
        assert_eq!(
            bound(-10., -10., 10., 10.),
            circle.get_bound()
        )
    }

    fn test_circle(dx: f32, dy: f32, px: f32, py: f32, rot: f32) {
        let circle = Circle::new(10.);
        let direction = Vector2::new(dx, dy);
        let transform = Decomposed {
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
        let r = Rectangle::new(10., 10.);
        assert_eq!(
            bound(-5., -5., 5., 5.),
            r.get_bound()
        )
    }

    // convex polygon
    // not testing bound as ::util::get_bound is fairly well tested
    // not testing far point as ::util::get_max_point is rigorously tested

    // util
    fn bound(min_x : f32, min_y : f32, max_x : f32, max_y : f32) -> Aabb2<f32> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
