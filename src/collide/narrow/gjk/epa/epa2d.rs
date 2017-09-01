use cgmath::{Vector2, Point2};
use cgmath::num_traits::Float;
use collision::Aabb2;

use super::*;
use {Pose, Real};
use collide::{CollisionPrimitive, Contact, CollisionStrategy};
use collide::narrow::gjk::support;
use collide::primitive2d::Primitive2D;

pub struct EPA2D;

impl<T> EPA<T> for EPA2D
where
    T: Pose<Point2<Real>>,
{
    type Vector = Vector2<Real>;
    type Point = Point2<Real>;
    type Aabb = Aabb2<Real>;
    type Primitive = Primitive2D;

    fn process(
        &self,
        simplex: &mut Vec<SupportPoint<Point2<Real>>>,
        left: &CollisionPrimitive<Primitive2D, Aabb2<Real>, T>,
        left_transform: &T,
        right: &CollisionPrimitive<Primitive2D, Aabb2<Real>, T>,
        right_transform: &T,
    ) -> Vec<Contact<Vector2<Real>>> {
        let mut i = 0;
        if closest_edge(&simplex).is_none() {
            return Vec::default();
        }

        loop {
            let e = closest_edge(&simplex);
            let e = e.unwrap();
            let p = support(left, left_transform, right, right_transform, &e.normal);
            let d = p.v.dot(e.normal);
            if d - e.distance < EPA_TOLERANCE {
                return vec![
                    Contact::new_impl(
                        CollisionStrategy::FullResolution,
                        e.normal,
                        e.distance
                    ),
                ];
            } else {
                simplex.insert(e.index, p);
            }
            i += 1;
            if i >= MAX_ITERATIONS {
                return vec![
                    Contact::new_impl(
                        CollisionStrategy::FullResolution,
                        e.normal,
                        e.distance
                    ),
                ];
            }
        }
    }

    fn new() -> Self {
        Self {}
    }
}

#[derive(Debug)]
struct Edge {
    pub normal: Vector2<Real>,
    pub distance: Real,
    pub index: usize,
}

impl Edge {
    pub fn new() -> Self {
        Self {
            normal: Vector2::zero(),
            distance: Real::infinity(),
            index: 0,
        }
    }
}

fn closest_edge(simplex: &Vec<SupportPoint<Point2<Real>>>) -> Option<Edge> {
    if simplex.len() < 3 {
        None
    } else {
        let mut edge = Edge::new();
        for i in 0..simplex.len() {
            let j = if i + 1 == simplex.len() { 0 } else { i + 1 };
            let a = simplex[i].v;
            let b = simplex[j].v;
            let e = b - a;
            let oa = a;
            let n = ::util::triple_product(&e, &oa, &e).normalize();
            let d = n.dot(a);
            if d < edge.distance {
                edge.distance = d;
                edge.normal = n;
                edge.index = j;
            }
        }
        Some(edge)
    }
}
