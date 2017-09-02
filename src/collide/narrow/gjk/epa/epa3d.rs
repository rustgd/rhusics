use cgmath::{Vector3, Point3};
use collision::Aabb3;

use super::*;
use {Pose, Real};
use collide::{CollisionPrimitive, Contact, CollisionStrategy};
use collide::narrow::gjk::{support, SupportPoint};
use collide::primitive3d::Primitive3D;

pub struct EPA3D;

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
        mut simplex: &mut Vec<SupportPoint<Point3<Real>>>,
        left: &CollisionPrimitive<Primitive3D, Aabb3<Real>, T>,
        left_transform: &T,
        right: &CollisionPrimitive<Primitive3D, Aabb3<Real>, T>,
        right_transform: &T,
    ) -> Vec<Contact<Vector3<Real>>> {
        let mut polytope = Polytope::new(&mut simplex);
        let mut i = 1;
        loop {
            let p = {
                let face = polytope.closest_face_to_origin();
                let p = support(left, left_transform, right, right_transform, &face.normal);
                let d = p.v.dot(face.normal);
                if d - face.distance < EPA_TOLERANCE || i >= MAX_ITERATIONS {
                    return contact(face);
                }
                p
            };
            polytope.add(p);
            i += 1;
        }
    }

    fn new() -> Self {
        Self {}
    }
}

#[allow(unused_variables)]
fn contact(face: &Face) -> Vec<Contact<Vector3<Real>>> {
    vec![
        Contact::new_impl(
            CollisionStrategy::FullResolution,
            face.normal.clone(),
            face.distance
        ),
    ]
}

struct Face {
    pub vertices: [usize; 3],
    pub normal: Vector3<Real>,
    pub distance: Real,
}

impl Face {
    fn new_impl(simplex: &Vec<SupportPoint<Point3<Real>>>, a: usize, b: usize, c: usize) -> Self {
        let ab = simplex[b].v - simplex[a].v;
        let ac = simplex[c].v - simplex[a].v;
        let normal = ab.cross(ac).normalize();
        let distance = normal.dot(simplex[a].v);
        Self {
            vertices: [a, b, c],
            normal,
            distance,
        }
    }

    pub fn new(simplex: &Vec<SupportPoint<Point3<Real>>>) -> Vec<Self> {
        vec![
            Self::new_impl(simplex, 3, 2, 1), // ABC
            Self::new_impl(simplex, 3, 1, 0), // ACD
            Self::new_impl(simplex, 3, 0, 2), // ADB
            Self::new_impl(simplex, 2, 0, 1), // BDC
        ]
    }
}

struct Polytope<'a> {
    vertices: &'a mut Vec<SupportPoint<Point3<Real>>>,
    faces: Vec<Face>,
}

impl<'a> Polytope<'a> {
    pub fn new(simplex: &'a mut Vec<SupportPoint<Point3<Real>>>) -> Self {
        let faces = Face::new(simplex);
        Self {
            vertices: simplex,
            faces,
        }
    }

    pub fn closest_face_to_origin(&'a self) -> &'a Face {
        let mut face = &self.faces[0];
        for f in self.faces[1..].iter() {
            if f.distance < face.distance {
                face = f;
            }
        }
        face
    }

    pub fn add(&mut self, sup: SupportPoint<Point3<Real>>) {

        // remove faces that can see the point
        let mut edges = Vec::default();
        let mut i = 0;
        while i < self.faces.len() {
            let dot = self.faces[i].normal.dot(
                sup.v -
                    self.vertices[self.faces[i]
                                      .vertices
                                      [0]]
                        .v,
            );
            if dot > 0. {
                let face = self.faces.swap_remove(i);
                remove_or_add_edge(&mut edges, (face.vertices[0], face.vertices[1]));
                remove_or_add_edge(&mut edges, (face.vertices[1], face.vertices[2]));
                remove_or_add_edge(&mut edges, (face.vertices[2], face.vertices[0]));
            } else {
                i += 1;
            }
        }

        // add vertex
        let n = self.vertices.len();
        self.vertices.push(sup);

        // add new faces
        for (a, b) in edges {
            self.faces.push(Face::new_impl(&self.vertices, n, a, b));
        }
    }
}

#[inline]
fn remove_or_add_edge(edges: &mut Vec<(usize, usize)>, edge: (usize, usize)) {
    for i in 0..edges.len() {
        if edge.0 == edges[i].1 && edge.1 == edges[i].0 {
            edges.remove(i);
            return;
        }
    }
    edges.push(edge);
}
