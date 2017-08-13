use cgmath::{Vector2, Point2, InnerSpace};
use cgmath;
use std::ops::Neg;
use super::{CollisionPrimitive, CollisionShape, CollisionEvent};
use super::super::BodyPose;
use collision::Discrete;

use std;

const EPA_TOLERANCE : f32 = 0.00001;
const MAX_ITERATIONS : usize = 100;


pub trait NarrowPhase {
    fn collide(&mut self,
               left: &(CollisionShape, BodyPose),
               right: &(CollisionShape, BodyPose)) -> Option<CollisionEvent>;
}

pub struct GJK;

impl GJK {
    fn gjk(left: &CollisionPrimitive,
           left_pose: &BodyPose,
           right: &CollisionPrimitive,
           right_pose: &BodyPose) -> Option<Vec<Vector2<f32>>> {
        let mut d = right_pose.position - left_pose.position;
        let mut simplex : Vec<Vector2<f32>> = Vec::default();
        simplex.push(Self::support(left, left_pose, right, right_pose, &d));
        if cgmath::dot(*simplex.last().unwrap(), d) <= 0. {
            return None;
        }
        d = d.neg();
        let mut i = 0;
        loop {
            simplex.push(Self::support(left, left_pose, right, right_pose, &d));
            if cgmath::dot(*simplex.last().unwrap(), d) <= 0. {
                return None;
            } else if Self::process_simplex(&mut simplex, &mut d) {
                return Some(simplex);
            }
            i += 1;
            if i >= MAX_ITERATIONS {
                return None;
            }
        }
    }

    fn support(left : &CollisionPrimitive,
               left_pose: &BodyPose,
               right : &CollisionPrimitive,
               right_pose: &BodyPose,
               direction: &Vector2<f32>) -> Vector2<f32> {
        left.primitive.get_far_point(direction, &left.offset, &left_pose)
            - right.primitive.get_far_point(&direction.neg(), &right.offset, &right_pose)
    }

    fn process_simplex(simplex: &mut Vec<Vector2<f32>>, d : &mut Vector2<f32>) -> bool {
        if simplex.len() == 3 {
            let a = simplex[2];
            let ao = a.neg();
            let b = simplex[1];
            let c = simplex[0];
            let ab = b - a;
            let ac = c - a;
            let ab_perp = Self::triple_product(&ac, &ab, &ab);
            if cgmath::dot(ab_perp, ao) > 0. {
                simplex.remove(0);
                *d = ab_perp;
            } else {
                let ac_perp = Self::triple_product(&ab, &ac, &ac);
                if cgmath::dot(ac_perp, ao) > 0. {
                    simplex.remove(1);
                    *d = ac_perp;
                } else {
                    return true;
                }
            }
        } else {
            let a = simplex[1];
            let ao = a.neg();
            let b = simplex[0];
            let ab = b - a;
            *d = Self::triple_product(&ab, &ao, &ab);
        }
        false
    }

    #[inline]
    fn triple_product(a : &Vector2<f32>, b : &Vector2<f32>, c : &Vector2<f32>) -> Vector2<f32> {
        let ac = a.x * c.x + a.y * c.y;
        let bc = b.x * c.x + b.y * c.y;
        Vector2::new(
            b.x * ac - a.x * bc,
            b.y * ac - a.y * bc,
        )
    }

    fn epa(bodies: (usize, usize),
           mut simplex : Vec<Vector2<f32>>,
           left: &CollisionPrimitive,
           left_pose: &BodyPose,
           right: &CollisionPrimitive,
           right_pose: &BodyPose) -> CollisionEvent {
        loop {
            let (normal, distance, index) = Self::find_closest_edge(&simplex);
            let v = Self::support(left, left_pose, right, right_pose, &normal);
            let d = cgmath::dot(v, normal);
            if d - distance < EPA_TOLERANCE {
                return CollisionEvent::new_impl(bodies,
                                                normal,
                                                d);
            } else {
                simplex.insert(index, v);
            }
        }
    }

    fn find_closest_edge(simplex: &Vec<Vector2<f32>>) -> (Vector2<f32>, f32, usize){
        let mut distance = std::f32::MAX;
        let mut normal = Vector2::new(0., 0.);
        let mut index = 0;
        for i in 0..simplex.len() {
            let j = if (i + 1) == simplex.len() { 0 } else { (i + 1) };
            let a = simplex[i];
            let b = simplex[j];
            let e = b - a;
            let oa = a;
            let n = Self::triple_product(&e, &oa, &e);
            n.normalize();
            let d = cgmath::dot(n, a);
            if d < distance {
                distance = d;
                index = j;
                normal = n;
            }
        }
        (normal, distance, index)
    }
}

impl NarrowPhase for GJK {
    fn collide(&mut self,
               &(ref left, ref left_pose): &(CollisionShape, BodyPose),
               &(ref right, ref right_pose): &(CollisionShape, BodyPose)) -> Option<CollisionEvent> {
        if !left.enabled || !right.enabled ||
            left.primitives.is_empty() || right.primitives.is_empty() {
            return None;
        }

        // FIXME: sort primitives on distance to the other shape ?
        for left_primitive in &left.primitives {
            for right_primitive in &right.primitives {
                if (left.transformed_bound, right.transformed_bound).intersects() {
                    match Self::gjk(&left_primitive, left_pose, &right_primitive, right_pose) {
                        Some(simplex) => return Some(Self::epa((left.id, right.id),
                                                               simplex,
                                                               &left_primitive,
                                                               left_pose,
                                                               &right_primitive,
                                                               right_pose)),
                        None => ()
                    };
                }
            }
        }
        None
    }
}
