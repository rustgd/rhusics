use std::collections::HashMap;

use cgmath::{Point3, Vector3};
use cgmath::num_traits::Float;
use cgmath::prelude::*;
use collision::Aabb3;
use collision::Ray3;
use collision::prelude::*;

use {Pose, Real};

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
    normal: Vector3<Real>,
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
            bound: vertices
                .iter()
                .fold(Aabb3::zero(), |bound, p| bound.grow(*p)),
        }
    }

    /// Create a new convex polyhedron from the given vertices and faces.
    pub fn new_with_faces(vertices: Vec<Point3<Real>>, faces: Vec<(usize, usize, usize)>) -> Self {
        let (vertices, edges, faces) = build_half_edges(&vertices, &faces);
        Self {
            mode: PolytopeMode::HalfEdge,
            bound: vertices
                .iter()
                .fold(Aabb3::zero(), |bound, p| bound.grow(p.position)),
            vertices,
            edges,
            faces,
        }
    }

    #[inline]
    /// Get the AABB for the polytope
    pub fn get_bound(&self) -> Aabb3<Real> {
        self.bound.clone()
    }

    #[inline]
    /// Get the furthest point from the origin on the shape in a given direction.
    pub fn get_far_point<T>(&self, direction: &Vector3<Real>, transform: &T) -> Point3<Real>
    where
        T: Pose<Point3<Real>>,
    {
        let p = match self.mode {
            PolytopeMode::VertexOnly => {
                self.brute_force_far_point(transform.inverse_rotation().rotate_vector(*direction))
            }

            PolytopeMode::HalfEdge => {
                self.hill_climb_far_point(transform.inverse_rotation().rotate_vector(*direction))
            }
        };
        *transform.position() + transform.rotation().rotate_point(p).to_vec()
    }

    #[inline]
    fn brute_force_far_point(&self, direction: Vector3<Real>) -> Point3<Real> {
        let (p, _) = self.vertices
            .iter()
            .map(|v| (v.position, v.position.dot(direction)))
            .fold(
                (Point3::from_value(0.), Real::neg_infinity()),
                |(max_p, max_dot), (v, dot)| if dot > max_dot {
                    (v.clone(), dot)
                } else {
                    (max_p, max_dot)
                },
            );
        p
    }

    #[inline]
    fn hill_climb_far_point(&self, direction: Vector3<Real>) -> Point3<Real> {
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
        let mut face = Face {
            edge: 0,
            normal: (vertices[c].position - vertices[a].position)
                .cross(vertices[b].position - vertices[a].position)
                .normalize(),
            ready: false,
        };
        let face_index = faces.len();
        let mut face_edge_indices = [0, 0, 0];
        for j in 0..3 {
            let i = if j == 0 { 2 } else { j - 1 };
            let v0 = face_vertices[i];
            let v1 = face_vertices[j];

            let (edge_v0_v1_index, edge_v1_v0_index) = if let Some(edge) = edge_map.get(&(v0, v1)) {
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

impl Discrete<Ray3<Real>> for ConvexPolytope {
    fn intersects(&self, ray: &Ray3<Real>) -> bool {
        false // TODO
    }
}

/// Compute intersection point of ray and face in barycentric coordinates.
fn intersect_ray_face(
    ray: &Ray3<Real>,
    polytope: &ConvexPolytope,
    face: &Face,
) -> (Real, Real, Real) {
    (0., 0., 0.) // TODO
}

#[cfg(test)]
mod tests {

    use cgmath::{Point3, Transform, Vector3};

    use super::ConvexPolytope;
    use Real;
    use collide3d::BodyPose3;

    #[test]
    fn test_polytope_half_edge() {
        let vertices = vec![
            Point3::<Real>::new(0., 1., 0.),
            Point3::<Real>::new(0., -1., 1.),
            Point3::<Real>::new(-1., -1., -1.),
            Point3::<Real>::new(1., -1., -1.),
        ];
        let faces = vec![(0, 1, 2), (0, 2, 3), (0, 3, 1), (3, 2, 1)];

        let polytope_with_faces = ConvexPolytope::new_with_faces(vertices.clone(), faces);
        let polytope = ConvexPolytope::new(vertices);

        println!("{:?}", polytope_with_faces);

        let transform = BodyPose3::one();

        let direction = Vector3::new(1., 0., 0.);
        assert_eq!(
            Point3::new(1., -1., -1.),
            polytope.get_far_point(&direction, &transform)
        );
        assert_eq!(
            Point3::new(1., -1., -1.),
            polytope_with_faces.get_far_point(&direction, &transform)
        );

        let direction = Vector3::new(0., 1., 0.);
        assert_eq!(
            Point3::new(0., 1., 0.),
            polytope.get_far_point(&direction, &transform)
        );
        assert_eq!(
            Point3::new(0., 1., 0.),
            polytope_with_faces.get_far_point(&direction, &transform)
        );
    }
}
