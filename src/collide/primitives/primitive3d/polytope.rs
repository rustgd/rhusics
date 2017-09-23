use std::collections::HashMap;

use bit_set::BitSet;
use cgmath::{Point3, Vector3};
use cgmath::num_traits::Float;
use cgmath::prelude::*;
use collision::{Aabb3, Ray3, Plane};
use collision::prelude::*;

use {Pose, Real};
use collide::primitives::{HasAABB, SupportFunction, ContinuousTransformed, DiscreteTransformed};

#[derive(Debug, Clone)]
enum PolytopeMode {
    VertexOnly,
    HalfEdge,
}

#[derive(Debug, Clone)]
struct Vertex {
    position: Point3<Real>,
    edge: usize,
    ready: bool,
}

#[derive(Debug, Clone)]
struct Edge {
    target_vertex: usize,
    left_face: usize,
    next_edge: usize,
    previous_edge: usize,
    twin_edge: usize,
    ready: bool,
}

#[derive(Debug, Clone)]
struct Face {
    edge: usize,
    vertices: (usize, usize, usize),
    plane: Plane<Real>,
    ready: bool,
}

/// Convex polyhedron primitive.
///
/// Can contain any number of vertices, but a high number of vertices will
/// affect performance of course. It is recommended for high vertex counts, to also provide the
/// faces, this will cause the support function to use hill climbing on a half edge structure,
/// resulting in better performance. The breakpoint is around 250 vertices, but the face version is
/// only marginally slower on lower vertex counts (about 1-2%), while for higher vertex counts it's
/// about 2-5 times faster.
#[derive(Debug, Clone)]
pub struct ConvexPolytope {
    mode: PolytopeMode,
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    faces: Vec<Face>,
    bound: Aabb3<Real>,
}

impl ConvexPolytope {
    /// Create a new convex polyhedron from the given vertices.
    pub fn new(vertices: Vec<Point3<Real>>) -> Self {
        Self {
            mode: PolytopeMode::VertexOnly,
            vertices: vertices
                .iter()
                .map(|v| {
                    Vertex {
                        position: v.clone(),
                        edge: 0,
                        ready: true,
                    }
                })
                .collect(),
            edges: Vec::default(),
            faces: Vec::default(),
            bound: vertices.iter().fold(
                Aabb3::zero(),
                |bound, p| bound.grow(*p),
            ),
        }
    }

    /// Create a new convex polyhedron from the given vertices and faces.
    pub fn new_with_faces(vertices: Vec<Point3<Real>>, faces: Vec<(usize, usize, usize)>) -> Self {
        let (vertices, edges, faces) = build_half_edges(&vertices, &faces);
        Self {
            mode: PolytopeMode::HalfEdge,
            bound: vertices.iter().fold(Aabb3::zero(), |bound, p| {
                bound.grow(p.position)
            }),
            vertices,
            edges,
            faces,
        }
    }

    #[inline]
    fn brute_force_support_point(&self, direction: Vector3<Real>) -> Point3<Real> {
        let (p, _) = self.vertices
            .iter()
            .map(|v| (v.position, v.position.dot(direction)))
            .fold((Point3::from_value(0.), Real::neg_infinity()), |(max_p,
              max_dot),
             (v, dot)| {
                if dot > max_dot {
                    (v.clone(), dot)
                } else {
                    (max_p, max_dot)
                }
            });
        p
    }

    #[inline]
    fn hill_climb_support_point(&self, direction: Vector3<Real>) -> Point3<Real> {
        let mut best_index = 0;
        let mut best_dot = self.vertices[best_index].position.dot(direction);

        loop {
            let previous_best_dot = best_dot;
            let mut edge_index = self.vertices[best_index].edge;
            let start_edge_index = edge_index;

            loop {
                let vertex_index = self.edges[edge_index].target_vertex;

                let dot = self.vertices[vertex_index].position.dot(direction);
                if dot > best_dot {
                    best_index = vertex_index;
                    best_dot = dot;
                }

                edge_index = self.edges[self.edges[edge_index].twin_edge].next_edge;
                if start_edge_index == edge_index {
                    break;
                }
            }

            if best_dot == previous_best_dot {
                break;
            }
        }

        self.vertices[best_index].position.clone()
    }
}

/// Create half edge data structure from vertices and faces
fn build_half_edges(
    vertices: &Vec<Point3<Real>>,
    in_faces: &Vec<(usize, usize, usize)>,
) -> (Vec<Vertex>, Vec<Edge>, Vec<Face>) {
    let mut vertices: Vec<Vertex> = vertices
        .iter()
        .map(|v| {
            Vertex {
                position: v.clone(),
                edge: 0,
                ready: false,
            }
        })
        .collect();
    let mut edges: Vec<Edge> = vec![];
    let mut faces: Vec<Face> = vec![];
    let mut edge_map: HashMap<(usize, usize), usize> = HashMap::default();
    for &(a, b, c) in in_faces {
        let face_vertices = [a, b, c];
        println!("{}, {}, {}", a, b, c);
        println!("{:?}, {:?}, {:?}", vertices[a], vertices[b], vertices[c]);
        let mut face = Face {
            edge: 0,
            vertices: (a, b, c),
            plane: Plane::from_points(
                vertices[a].position,
                vertices[b].position,
                vertices[c].position,
            ).unwrap(),
            ready: false,
        };
        println!("{:?}", face);
        let face_index = faces.len();
        let mut face_edge_indices = [0, 0, 0];
        for j in 0..3 {
            let i = if j == 0 { 2 } else { j - 1 };
            let v0 = face_vertices[i];
            let v1 = face_vertices[j];

            let (edge_v0_v1_index, edge_v1_v0_index) =
                if let Some(edge) = edge_map.get(&(v0, v1)) {
                    (edge.clone(), edges[*edge].twin_edge)
                } else {
                    let edge_v0_v1_index = edges.len();
                    let edge_v1_v0_index = edges.len() + 1;

                    edges.push(Edge {
                        target_vertex: v1,
                        left_face: 0,
                        next_edge: 0,
                        previous_edge: 0,
                        twin_edge: edge_v1_v0_index,
                        ready: false,
                    });

                    edges.push(Edge {
                        target_vertex: v0,
                        left_face: 0,
                        next_edge: 0,
                        previous_edge: 0,
                        twin_edge: edge_v0_v1_index,
                        ready: false,
                    });

                    (edge_v0_v1_index, edge_v1_v0_index)
                };

            edge_map.insert((v0, v1), edge_v0_v1_index);
            edge_map.insert((v1, v0), edge_v1_v0_index);

            if !edges[edge_v0_v1_index].ready {
                edges[edge_v0_v1_index].left_face = face_index;
                edges[edge_v0_v1_index].ready = true;
            }

            if !vertices[v0].ready {
                vertices[v0].edge = edge_v0_v1_index;
                vertices[v0].ready = true;
            }

            if !face.ready {
                face.edge = edge_v0_v1_index;
                face.ready = true;
            }

            face_edge_indices[i] = edge_v0_v1_index;
        }

        faces.push(face);

        for j in 0..3 {
            let i = if j == 0 { 2 } else { j - 1 };
            let edge_i = face_edge_indices[i];
            let edge_j = face_edge_indices[j];
            edges[edge_i].next_edge = edge_j;
            edges[edge_j].previous_edge = edge_i;
        }
    }

    (vertices, edges, faces)
}

impl SupportFunction for ConvexPolytope {
    type Point = Point3<Real>;

    fn support_point<T>(&self, direction: &Vector3<Real>, transform: &T) -> Point3<Real>
    where
        T: Pose<Point3<Real>>,
    {
        let p = match self.mode {
            PolytopeMode::VertexOnly => {
                self.brute_force_support_point(
                    transform.inverse_rotation().rotate_vector(*direction),
                )
            }

            PolytopeMode::HalfEdge => {
                self.hill_climb_support_point(
                    transform.inverse_rotation().rotate_vector(*direction),
                )
            }
        };
        transform.transform_point(p)
    }
}

impl HasAABB for ConvexPolytope {
    type Aabb = Aabb3<Real>;

    fn get_bound(&self) -> Aabb3<Real> {
        self.bound.clone()
    }
}

impl DiscreteTransformed<Ray3<Real>> for ConvexPolytope {
    type Point = Point3<Real>;

    fn intersects_transformed<T>(&self, ray: &Ray3<Real>, transform: &T) -> bool
    where
        T: Transform<Point3<Real>>,
    {
        self.intersects(&ray.transform(transform.inverse_transform().unwrap()))
    }
}

/// TODO: better algorithm for finding faces to intersect with?
impl Discrete<Ray3<Real>> for ConvexPolytope {
    /// Ray must be in object space
    fn intersects(&self, ray: &Ray3<Real>) -> bool {
        let mut face_index = 0;
        let mut checked = BitSet::with_capacity(self.faces.len());
        while face_index < self.faces.len() {
            checked.insert(face_index);
            let f = &self.faces[face_index];
            match intersect_ray_face(ray, &self, &f) {
                Some((u, v, w)) => {
                    if in_range(u) && in_range(v) && in_range(w) {
                        return true;
                    } else {
                        face_index = next_face_classify(self, face_index, Some((u, v, w)), &mut checked);
                    }
                }
                _ => {
                    face_index = next_face_classify(self, face_index, None, &mut checked);
                },
            }
        }
        false
    }
}

impl ContinuousTransformed<Ray3<Real>> for ConvexPolytope {
    type Point = Point3<Real>;
    type Result = Point3<Real>;

    fn intersection_transformed<T>(&self, ray: &Ray3<Real>, transform: &T) -> Option<Point3<Real>>
    where
        T: Transform<Point3<Real>>,
    {
        self.intersection(&ray.transform(transform.inverse_transform().unwrap()))
            .map(|p| transform.transform_point(p))
    }
}

// TODO: better algorithm for finding faces to intersect with?
impl Continuous<Ray3<Real>> for ConvexPolytope {
    type Result = Point3<Real>;

    /// Ray must be in object space
    fn intersection(&self, ray: &Ray3<Real>) -> Option<Point3<Real>> {
        let mut face_index = 0;
        let mut checked = BitSet::with_capacity(self.faces.len());
        while face_index < self.faces.len() {
            checked.insert(face_index);
            let f = &self.faces[face_index];
            match intersect_ray_face(ray, &self, &f) {
                Some((u, v, w)) => {
                    println!("face v: {}, {}, {}", u, v, w);
                    if in_range(u) && in_range(v) && in_range(w) {
                        let v0 = f.vertices.0;
                        let v1 = f.vertices.1;
                        let v2 = f.vertices.2;
                        let p = (self.vertices[v0].position * u) +
                            (self.vertices[v1].position.to_vec() * v) +
                            (self.vertices[v2].position.to_vec() * w);
                        return Some(p);
                    }
                    face_index = next_face_classify(self, face_index, Some((u, v, w)), &mut checked);
                }
                _ => {
                    face_index = next_face_classify(self, face_index, None, &mut checked);
                },
            }

        }
        None
    }
}

#[inline]
fn in_range(v: Real) -> bool {
    v >= 0. && v <= 1.
}

#[inline]
fn next_face_classify(polytope: &ConvexPolytope, face_index: usize, bary_coords: Option<(Real, Real, Real)>, checked: &mut BitSet) -> usize {
    if polytope.faces.len() < 10 {
        face_index + 1
    } else {
        match bary_coords {
            None => {
                let mut next = face_index + 1;
                while next < polytope.faces.len() && checked.contains(next) {
                    next = next + 1;
                }
                next
            }

            Some((u, v, _)) => {
                let face = &polytope.faces[face_index];
                let target_vertex_index = if u < 0. {
                    face.vertices.2
                } else if v < 0. {
                    face.vertices.0
                } else {
                    face.vertices.1
                };

                let face_edge = &polytope.edges[face.edge];

                let edges = if face_edge.target_vertex == target_vertex_index {
                    [face.edge, face_edge.next_edge, face_edge.previous_edge]
                } else if polytope.edges[face_edge.previous_edge].target_vertex == target_vertex_index {
                    [face_edge.previous_edge, face.edge, face_edge.next_edge]
                } else {
                    [face_edge.next_edge, face_edge.previous_edge, face.edge]
                };

                for edge_index in edges.iter() {
                    let twin_edge = polytope.edges[*edge_index].twin_edge;
                    if !checked.contains(polytope.edges[twin_edge].left_face) {
                        return polytope.edges[twin_edge].left_face;
                    }
                }

                // TODO: if none of the closest faces are intersecting, and we got here, that means this face is probably the closest to an intersection?

                for i in 0..polytope.faces.len() {
                    if !checked.contains(i) {
                        return i
                    }
                }

                polytope.faces.len()
            }
        }
    }
}

/// Compute intersection point of ray and face in barycentric coordinates.
#[inline]
fn intersect_ray_face(
    ray: &Ray3<Real>,
    polytope: &ConvexPolytope,
    face: &Face,
) -> Option<(Real, Real, Real)> {
    println!("{:?} {:?}", ray, face);

    let n_dir = face.plane.n.dot(ray.direction);
    println!("normal * ray_dir: {}", n_dir);
    if n_dir < 0. {
        let v0 = face.vertices.0;
        let v1 = face.vertices.1;
        let v2 = face.vertices.2;
        face.plane.intersection(ray).map(|p| {
            ::util::barycentric_point(
                p,
                polytope.vertices[v0].position,
                polytope.vertices[v1].position,
                polytope.vertices[v2].position,
            )
        })
    } else {
        None
    }
}

#[cfg(test)]
mod tests {

    use cgmath::{Point3, Vector3, Quaternion, Rad};
    use cgmath::prelude::*;
    use collision::{Aabb3, Ray3};
    use collision::prelude::*;

    use super::ConvexPolytope;
    use Real;
    use collide::primitives::{SupportFunction, HasAABB, DiscreteTransformed, ContinuousTransformed};
    use collide3d::BodyPose3;

    #[test]
    fn test_polytope_half_edge() {
        let vertices = vec![
            Point3::<Real>::new(1., 0., 0.),
            Point3::<Real>::new(0., 1., 0.),
            Point3::<Real>::new(0., 0., 1.),
            Point3::<Real>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope_with_faces = ConvexPolytope::new_with_faces(vertices.clone(), faces);
        let polytope = ConvexPolytope::new(vertices);

        let transform = BodyPose3::one();

        let direction = Vector3::new(1., 0., 0.);
        assert_eq!(
            Point3::new(1., 0., 0.),
            polytope.support_point(&direction, &transform)
        );
        assert_eq!(
            Point3::new(1., 0., 0.),
            polytope_with_faces.support_point(&direction, &transform)
        );

        let direction = Vector3::new(0., 1., 0.);
        assert_eq!(
            Point3::new(0., 1., 0.),
            polytope.support_point(&direction, &transform)
        );
        assert_eq!(
            Point3::new(0., 1., 0.),
            polytope_with_faces.support_point(&direction, &transform)
        );
    }

    #[test]
    fn test_polytope_bound() {
        let vertices = vec![
            Point3::<Real>::new(1., 0., 0.),
            Point3::<Real>::new(0., 1., 0.),
            Point3::<Real>::new(0., 0., 1.),
            Point3::<Real>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolytope::new_with_faces(vertices.clone(), faces);
        assert_eq!(
            Aabb3::new(Point3::new(0., 0., 0.), Point3::new(1., 1., 1.)),
            polytope.get_bound()
        );
    }

    #[test]
    fn test_ray_discrete() {
        let vertices = vec![
            Point3::<Real>::new(1., 0., 0.),
            Point3::<Real>::new(0., 1., 0.),
            Point3::<Real>::new(0., 0., 1.),
            Point3::<Real>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolytope::new_with_faces(vertices.clone(), faces);
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        assert!(polytope.intersects(&ray));
        let ray = Ray3::new(Point3::new(0.5, 5., 0.5), Vector3::new(0., 1., 0.));
        assert!(!polytope.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let vertices = vec![
            Point3::<Real>::new(1., 0., 0.),
            Point3::<Real>::new(0., 1., 0.),
            Point3::<Real>::new(0., 0., 1.),
            Point3::<Real>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolytope::new_with_faces(vertices.clone(), faces);
        let transform = BodyPose3::one();
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        assert!(polytope.intersects_transformed(&ray, &transform));
        let ray = Ray3::new(Point3::new(0.5, 5., 0.5), Vector3::new(0., 1., 0.));
        assert!(!polytope.intersects_transformed(&ray, &transform));
        let transform = BodyPose3::new(Point3::new(0., 1., 0.), Quaternion::one());
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        assert!(polytope.intersects_transformed(&ray, &transform));
        let transform = BodyPose3::new(Point3::new(0., 0., 0.), Quaternion::from_angle_z(Rad(0.3)));
        assert!(polytope.intersects_transformed(&ray, &transform));
    }

    #[test]
    fn test_ray_continuous() {
        let vertices = vec![
            Point3::<Real>::new(1., 0., 0.),
            Point3::<Real>::new(0., 1., 0.),
            Point3::<Real>::new(0., 0., 1.),
            Point3::<Real>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolytope::new_with_faces(vertices.clone(), faces);
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        let p = polytope.intersection(&ray).unwrap();
        assert_approx_eq!(0.25, p.x);
        assert_approx_eq!(0.5, p.y);
        assert_approx_eq!(0.25, p.z);
        let ray = Ray3::new(Point3::new(0.5, 5., 0.5), Vector3::new(0., 1., 0.));
        assert_eq!(None, polytope.intersection(&ray));
        let ray = Ray3::new(Point3::new(0., 5., 0.), Vector3::new(0., -1., 0.));
        assert_eq!(Some(Point3::new(0., 1., 0.)), polytope.intersection(&ray));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let vertices = vec![
            Point3::<Real>::new(1., 0., 0.),
            Point3::<Real>::new(0., 1., 0.),
            Point3::<Real>::new(0., 0., 1.),
            Point3::<Real>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolytope::new_with_faces(vertices.clone(), faces);
        let transform = BodyPose3::one();
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        let p = polytope.intersection_transformed(&ray, &transform).unwrap();
        assert_approx_eq!(0.25, p.x);
        assert_approx_eq!(0.5, p.y);
        assert_approx_eq!(0.25, p.z);
        let ray = Ray3::new(Point3::new(0.5, 5., 0.5), Vector3::new(0., 1., 0.));
        assert_eq!(None, polytope.intersection_transformed(&ray, &transform));
        let transform = BodyPose3::new(Point3::new(0., 1., 0.), Quaternion::one());
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        let p = polytope.intersection_transformed(&ray, &transform).unwrap();
        assert_approx_eq!(0.25, p.x);
        assert_approx_eq!(1.5, p.y);
        assert_approx_eq!(0.25, p.z);
        let transform = BodyPose3::new(Point3::new(0., 0., 0.), Quaternion::from_angle_z(Rad(0.3)));
        let p = polytope.intersection_transformed(&ray, &transform).unwrap();
        assert_approx_eq!(0.25, p.x);
        assert_approx_eq!(0.467716, p.y);
        assert_approx_eq!(0.25, p.z);
    }

    #[test]
    fn test_intersect_face() {
        let vertices = vec![
            Point3::<Real>::new(1., 0., 0.),
            Point3::<Real>::new(0., 1., 0.),
            Point3::<Real>::new(0., 0., 1.),
            Point3::<Real>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolytope::new_with_faces(vertices.clone(), faces);
        let ray = Ray3::new(Point3::new(1., -1., 1.), Vector3::new(0., 1., 0.));
        polytope.intersection(&ray);
    }
}
