use cgmath::{Vector3, Point3};
use collision::Aabb3;

use super::*;
use {Pose, Real};
use collide::{CollisionPrimitive, Contact, CollisionStrategy};
use collide::narrow::gjk::{support, SupportPoint};
use collide::primitive3d::Primitive3D;

/// EPA algorithm implementation for 3D. Only to be used in [`GJK`](struct.GJK.html).
#[derive(Debug)]
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
        if simplex.len() < 4 {
            return Vec::default();
        }
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

#[inline]
fn contact(face: &Face) -> Vec<Contact<Vector3<Real>>> {
    vec![
        Contact::new_impl(
            CollisionStrategy::FullResolution,
            face.normal.clone(),
            face.distance
        ),
    ]
}

#[derive(Debug)]
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

#[derive(Debug)]
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

#[cfg(test)]
mod tests {
    use cgmath::{Vector3, Quaternion, Rad};

    use super::*;
    use Real;
    use collide3d::*;

    #[test]
    fn test_remove_or_add_edge_added() {
        let mut edges = vec![(1, 2), (6, 5)];
        remove_or_add_edge(&mut edges, (4, 3));
        assert_eq!(3, edges.len());
        assert_eq!((4, 3), edges[2]);
    }

    #[test]
    fn test_remove_or_add_edge_removed() {
        let mut edges = vec![(1, 2), (6, 5)];
        remove_or_add_edge(&mut edges, (2, 1));
        assert_eq!(1, edges.len());
        assert_eq!((6, 5), edges[0]);
    }

    #[test]
    fn test_face_impl() {
        let simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let faces = Face::new(&simplex);
        assert_eq!(4, faces.len());
        assert_face(
            &faces[0],
            3,
            2,
            1,
            -0.8728715,
            0.43643576,
            0.21821788,
            1.0910894,
        );
        assert_face(&faces[1], 3, 1, 0, 0., -0.89442724, 0.44721362, 2.236068);
        assert_face(
            &faces[2],
            3,
            0,
            2,
            0.8728715,
            0.43643576,
            0.21821788,
            1.0910894,
        );
        assert_face(&faces[3], 2, 0, 1, 0., 0., -1., 1.0);
    }

    #[test]
    fn test_polytope_closest_to_origin() {
        let mut simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let polytope = Polytope::new(&mut simplex);
        let face = polytope.closest_face_to_origin();
        assert_face(face, 2, 0, 1, 0., 0., -1., 1.0);
    }

    #[test]
    fn test_polytope_add() {
        let mut simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let mut polytope = Polytope::new(&mut simplex);
        polytope.add(sup(0., 0., -2.));
        assert_eq!(5, polytope.vertices.len());
        assert_eq!(6, polytope.faces.len());
        assert_eq!([4, 2, 0], polytope.faces[3].vertices);
        assert_eq!([4, 0, 1], polytope.faces[4].vertices);
        assert_eq!([4, 1, 2], polytope.faces[5].vertices);
    }

    #[test]
    fn test_epa_3d() {
        let left = CollisionPrimitive3D::new(Box::new(10., 10., 10.).into());
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = CollisionPrimitive3D::new(Box::new(10., 10., 10.).into());
        let right_transform = transform_3d(7., 2., 0., 0.);
        let mut simplex = vec![
            sup(18., -12., 0.),
            sup(-2., 8., 0.),
            sup(-2., -12., 0.),
            sup(8., -2., -10.),
        ];
        let contacts = EPA3D.process(
            &mut simplex,
            &left,
            &left_transform,
            &right,
            &right_transform,
        );
        assert_eq!(1, contacts.len());
        assert_eq!(Vector3::new(-1., 0., 0.), contacts[0].normal);
        assert_eq!(2., contacts[0].penetration_depth);
    }

    fn assert_face(
        face: &Face,
        a: usize,
        b: usize,
        c: usize,
        nx: Real,
        ny: Real,
        nz: Real,
        d: Real,
    ) {
        assert_eq!([a, b, c], face.vertices);
        assert_approx_eq!(nx, face.normal.x);
        assert_approx_eq!(ny, face.normal.y);
        assert_approx_eq!(nz, face.normal.z);
        assert_approx_eq!(d, face.distance);
    }

    fn sup(x: Real, y: Real, z: Real) -> SupportPoint<Point3<Real>> {
        let mut s = SupportPoint::new();
        s.v = Vector3::new(x, y, z);
        s
    }

    fn transform_3d(x: Real, y: Real, z: Real, angle_z: Real) -> BodyPose3D {
        BodyPose3D::new(Point3::new(x, y, z), Quaternion::from_angle_z(Rad(angle_z)))
    }
}
