use cgmath::{Vector3, Point3};
use cgmath::num_traits::Float;
use collision::Aabb3;

use super::*;
use {Pose, Real};
use collide::{CollisionPrimitive, Contact, CollisionStrategy};
use collide::narrow::gjk::{support, SupportPoint};
use collide::primitive3d::Primitive3D;

pub struct EPA3D;

#[allow(unused_variables)]
impl<T> EPA<T> for EPA3D
where
    T: Pose<Point3<Real>>,
{
    type Vector = Vector3<Real>;
    type Point = Point3<Real>;
    type Aabb = Aabb3<Real>;
    type Primitive = Primitive3D;

    fn process(
        &self,
        simplex: &mut Vec<SupportPoint<Point3<Real>>>,
        left: &CollisionPrimitive<Primitive3D, Aabb3<Real>, T>,
        left_transform: &T,
        right: &CollisionPrimitive<Primitive3D, Aabb3<Real>, T>,
        right_transform: &T,
    ) -> Vec<Contact<Vector3<Real>>> {
        Vec::default() // TODO
    }

    fn new() -> Self {
        Self {}
    }
}

struct Face {
    vertices: [usize; 3],
}

struct Polytope {
    vertices: Vec<Vector3<Real>>,
    faces: Vec<Face>,
}
