use cgmath::{Vector2, Point2, InnerSpace, Array};
use cgmath;
use std;
use collision::Aabb2;
use super::{Primitive, BodyPose};


#[derive(Debug)]
pub struct Circle {
    pub radius: f32,
}

#[derive(Debug)]
pub struct Rectangle {
    pub dim: Vector2<f32>,
    half_dim: Vector2<f32>,
    corners: Vec<Vector2<f32>>,
}

impl Rectangle {
    pub fn new(dim: Vector2<f32>) -> Rectangle {
        Rectangle {
            dim,
            half_dim: dim / 2.,
            corners: Self::generate_corners(&dim),
        }
    }

    pub fn generate_corners(dimensions: &Vector2<f32>) -> Vec<Vector2<f32>> {
        vec![
            Vector2::new(dimensions.x / 2., dimensions.y / 2.),
            Vector2::new(-dimensions.x / 2., dimensions.y / 2.),
            Vector2::new(-dimensions.x / 2., -dimensions.y / 2.),
            Vector2::new(dimensions.x / 2., -dimensions.y / 2.),
        ]
    }
}

#[derive(Debug)]
pub struct ConvexPolygon {
    pub vertices: Vec<Vector2<f32>>,
}

impl Primitive for Circle {
    fn get_far_point(
        &self,
        direction: &Vector2<f32>,
        body_offset: &Vector2<f32>,
        pose: &BodyPose,
    ) -> Point2<f32> {
        pose.position + (pose.rotation * (direction.normalize_to(self.radius) + body_offset))
    }

    fn get_bound(&self) -> Aabb2<f32> {
        Aabb2::new(
            Point2::new(-self.radius, -self.radius),
            Point2::new(self.radius, self.radius),
        )
    }
}

impl Primitive for Rectangle {
    fn get_far_point(
        &self,
        direction: &Vector2<f32>,
        body_offset: &Vector2<f32>,
        pose: &BodyPose,
    ) -> Point2<f32> {
        get_max_point(&self.corners, direction, body_offset, pose)
    }

    fn get_bound(&self) -> Aabb2<f32> {
        Aabb2::new(
            Point2::new(-self.half_dim.x, -self.half_dim.y),
            Point2::new(self.half_dim.x, self.half_dim.y),
        )
    }
}

impl Primitive for ConvexPolygon {
    fn get_far_point(
        &self,
        direction: &Vector2<f32>,
        body_offset: &Vector2<f32>,
        pose: &BodyPose,
    ) -> Point2<f32> {
        get_max_point(&self.vertices, direction, body_offset, pose)
    }

    fn get_bound(&self) -> Aabb2<f32> {
        self.vertices.iter().fold(
            Aabb2::new(Point2::from_value(0.), Point2::from_value(0.)),
            |mut bound, p| {
                bound.min.x = p.x.min(bound.min.x);
                bound.max.x = p.x.max(bound.max.x);
                bound.min.y = p.y.min(bound.min.y);
                bound.max.y = p.y.max(bound.max.y);
                bound
            },
        )
    }
}

fn get_max_point(
    vertices: &Vec<Vector2<f32>>,
    direction: &Vector2<f32>,
    body_offset: &Vector2<f32>,
    pose: &BodyPose,
) -> Point2<f32> {
    let nd = pose.inverse_rotation * direction;
    let (p, _) = vertices
        .iter()
        .map(|v| v + body_offset)
        .map(|v| (v, cgmath::dot(Vector2::new(v.x, v.y), nd)))
        .fold((Vector2::new(0., 0.), std::f32::MIN), |(max_p, max_dot),
         (v, dot)| {
            if dot > max_dot {
                (v.clone(), dot)
            } else {
                (max_p, max_dot)
            }
        });
    pose.position + (pose.rotation * p)
}
