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
