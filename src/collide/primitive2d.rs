use cgmath::{Vector2, Point2, InnerSpace, BaseFloat, Decomposed, Transform, Basis2,
             EuclideanSpace, Array};
use collision::{Aabb2, Aabb};
use super::Primitive;

#[derive(Debug)]
pub struct Circle<S: BaseFloat> {
    pub radius: S,
}

#[derive(Debug)]
pub struct Rectangle<S: BaseFloat> {
    pub dim: Vector2<S>,
    half_dim: Vector2<S>,
    corners: Vec<Vector2<S>>,
}

impl<S: BaseFloat> Rectangle<S> {
    pub fn new(dim: Vector2<S>) -> Rectangle<S> {
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

impl<S> Primitive<S, Vector2<S>, Point2<S>, Decomposed<Vector2<S>, Basis2<S>>, Aabb2<S>>
    for Circle<S>
where
    S: BaseFloat + Send + Sync,
{
    fn get_far_point(
        &self,
        direction: &Vector2<S>,
        transform: &Decomposed<Vector2<S>, Basis2<S>>,
    ) -> Point2<S> {
        Point2::from_vec(transform.transform_vector(
            direction.normalize_to(self.radius),
        ))
    }

    fn get_bound(&self) -> Aabb2<S> {
        Aabb2::new(
            Point2::new(-self.radius, -self.radius),
            Point2::new(self.radius, self.radius),
        )
    }
}

impl<S> Primitive<S, Vector2<S>, Point2<S>, Decomposed<Vector2<S>, Basis2<S>>, Aabb2<S>>
    for Rectangle<S>
where
    S: BaseFloat + Send + Sync,
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

impl<S> Primitive<S, Vector2<S>, Point2<S>, Decomposed<Vector2<S>, Basis2<S>>, Aabb2<S>>
    for ConvexPolygon<S>
where
    S: BaseFloat + Send + Sync,
{
    fn get_far_point(
        &self,
        direction: &Vector2<S>,
        transform: &Decomposed<Vector2<S>, Basis2<S>>,
    ) -> Point2<S> {
        ::util::get_max_point(&self.vertices, direction, transform)
    }

    fn get_bound(&self) -> Aabb2<S> {
        self.vertices.iter().fold(
            Aabb2::new(Point2::from_value(S::zero()), Point2::from_value(S::zero())),
            |bound, p| bound.grow(Point2::from_vec(*p))
        )
    }
}
