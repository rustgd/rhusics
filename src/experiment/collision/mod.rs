pub mod narrow;
pub mod primitive;

pub use super::BodyPose;

use std::fmt::Debug;

use cgmath::{Vector2, Point2, Array, Zero};
use collision::{Aabb2, Aabb};

pub struct CollisionEvent {
    pub mode: CollisionMode,
    pub bodies: (usize, usize),
    pub normal: Vector2<f32>,
    pub penetration_depth: f32,
}

impl CollisionEvent {
    pub fn new(mode: CollisionMode, bodies: (usize, usize)) -> CollisionEvent {
        Self::new_impl(mode, bodies, Vector2::zero(), 0.)
    }

    pub fn new_impl(
        mode: CollisionMode,
        bodies: (usize, usize),
        normal: Vector2<f32>,
        penetration_depth: f32,
    ) -> CollisionEvent {
        CollisionEvent {
            mode,
            bodies,
            normal,
            penetration_depth,
        }
    }
}

pub trait Primitive: Debug + Send + Sync {
    fn get_far_point(
        &self,
        direction: &Vector2<f32>,
        body_offset: &Vector2<f32>,
        pose: &BodyPose,
    ) -> Point2<f32>;
    fn get_bound(&self) -> Aabb2<f32>;
}

#[derive(Debug)]
pub struct CollisionPrimitive {
    offset: Vector2<f32>,
    transformed_bound: Aabb2<f32>,
    base_bound: Aabb2<f32>,
    primitive: Box<Primitive>,
}

impl CollisionPrimitive {
    pub fn new<T: Primitive + 'static>(primitive: T) -> CollisionPrimitive {
        Self::new_impl(primitive, Vector2::zero())
    }

    pub fn new_impl<T: Primitive + 'static>(
        primitive: T,
        offset: Vector2<f32>,
    ) -> CollisionPrimitive {
        let bound = primitive.get_bound().add_v(offset);
        CollisionPrimitive {
            base_bound: bound.clone(),
            primitive: Box::new(primitive),
            transformed_bound: bound,
            offset,
        }
    }

    pub fn update_bound(&mut self, pose: &BodyPose) {
        if !pose.dirty {
            return;
        }
        self.transformed_bound = transform_bound(&self.base_bound, pose);
    }
}

#[derive(Debug, PartialEq)]
pub enum CollisionMode {
    FullResolution,
    CollisionOnly,
}

#[derive(Debug)]
pub struct CollisionShape {
    pub id: usize,
    pub enabled: bool,
    pub base_bound: Aabb2<f32>,
    pub transformed_bound: Aabb2<f32>,
    pub primitives: Vec<CollisionPrimitive>,
    pub mode: CollisionMode,
}

/*pub fn collide(shapes: &mut Vec<(CollisionShape, BodyPose)>,
               broad: &mut broad::BroadPhase,
               narrow: &mut narrow::NarrowPhase) -> Vec<CollisionEvent> {
    broad
        .compute(shapes)
        .iter()
        .filter_map(|&(left_index, right_index)|
            narrow.collide(&shapes[left_index], &shapes[right_index]))
        .collect()
}*/

impl CollisionShape {
    pub fn new(
        id: usize,
        mode: CollisionMode,
        primitives: Vec<CollisionPrimitive>,
    ) -> CollisionShape {
        let bound = get_bound(&primitives);
        CollisionShape {
            base_bound: bound.clone(),
            id,
            primitives,
            enabled: false,
            transformed_bound: bound,
            mode,
        }
    }

    pub fn update_bound(&mut self, pose: &BodyPose) {
        if !pose.dirty {
            return;
        }
        self.transformed_bound = transform_bound(&self.base_bound, pose);
        for mut primitive in &mut self.primitives {
            primitive.update_bound(pose);
        }
    }
}

fn get_bound(primitives: &Vec<CollisionPrimitive>) -> Aabb2<f32> {
    primitives.iter().map(|p| p.base_bound).fold(
        Aabb2::new(
            Point2::from_value(0.),
            Point2::from_value(0.),
        ),
        |mut bound, b| {
            bound.min.x = b.min.x.min(bound.min.x);
            bound.max.x = b.max.x.max(bound.max.x);
            bound.min.y = b.min.y.min(bound.min.y);
            bound.max.y = b.max.y.max(bound.max.y);
            bound
        },
    )
}

fn transform_bound(base: &Aabb2<f32>, pose: &BodyPose) -> Aabb2<f32> {
    base.to_corners()
        .iter()
        .map(|v| pose.rotation * Vector2::new(v.x, v.y))
        .fold(
            Aabb2::new(Point2::from_value(0.), Point2::from_value(0.)),
            |mut bound, p| {
                bound.min.x = p.x.min(bound.min.x);
                bound.max.x = p.x.max(bound.max.x);
                bound.min.y = p.y.min(bound.min.y);
                bound.max.y = p.y.max(bound.max.y);
                bound
            },
        )
        .add_v(Vector2::new(pose.position.x, pose.position.y))
}
