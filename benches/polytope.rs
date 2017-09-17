#![feature(test)]

extern crate test;
extern crate rhusics;
extern crate rand;
extern crate genmesh;
extern crate cgmath;

use cgmath::{Point3, Vector3};
use cgmath::prelude::*;
use rhusics::collide::primitives::primitive3d::ConvexPolytope;
use rhusics::collide3d::BodyPose3;
use genmesh::Triangulate;
use genmesh::generators::{SphereUV, SharedVertex, IndexedPolygon};
use rand::Rng;
use test::{Bencher, black_box};

#[bench]
fn test_polytope_10(bench: &mut Bencher) {
    test_polytope(bench, 10, false);
}

#[bench]
fn test_polytope_faces_10(bench: &mut Bencher) {
    test_polytope(bench, 10, true);
}

#[bench]
fn test_polytope_20(bench: &mut Bencher) {
    test_polytope(bench, 20, false);
}

#[bench]
fn test_polytope_faces_20(bench: &mut Bencher) {
    test_polytope(bench, 20, true);
}

#[bench]
fn test_polytope_30(bench: &mut Bencher) {
    test_polytope(bench, 30, false);
}

#[bench]
fn test_polytope_faces_30(bench: &mut Bencher) {
    test_polytope(bench, 30, true);
}

#[bench]
fn test_polytope_50(bench: &mut Bencher) {
    test_polytope(bench, 50, false);
}

#[bench]
fn test_polytope_faces_50(bench: &mut Bencher) {
    test_polytope(bench, 50, true);
}

#[bench]
fn test_polytope_500(bench: &mut Bencher) {
    test_polytope(bench, 500, false);
}

#[bench]
fn test_polytope_faces_500(bench: &mut Bencher) {
    test_polytope(bench, 500, true);
}

fn test_polytope(bench: &mut Bencher, n: usize, with_faces: bool) {
    let polytope = sphere(n, with_faces);
    let dirs = dirs(1000);
    let transform = BodyPose3::one();

    let mut i = 0;
    bench.iter(|| {
        black_box(polytope.get_far_point(&dirs[i % 1000], &transform));
        i += 1;
    });
}

fn dirs(n: usize) -> Vec<Vector3<f32>> {
    let mut rng = rand::thread_rng();
    (0..n)
        .map(|_| Vector3::new(rng.gen_range(-1., 1.),
                              rng.gen_range(-1., 1.),
                              rng.gen_range(-1., 1.)))
        .collect::<Vec<_>>()
}

fn sphere(n: usize, with_faces: bool) -> ConvexPolytope {
    let gen = SphereUV::new(n, n);

    let vertices = gen.shared_vertex_iter()
        .map(|v| Point3::from(v.pos))
        .collect::<Vec<_>>();

    if with_faces {
        let faces = gen.indexed_polygon_iter()
            .triangulate()
            .map(|f| (f.x, f.y, f.z))
            .collect::<Vec<_>>();

        ConvexPolytope::new_with_faces(vertices, faces)
    } else {
        ConvexPolytope::new(vertices)
    }
}
