#![feature(test)]

extern crate test;

use cgmath::prelude::*;
use cgmath::{Decomposed, Point3, Quaternion, Vector3};
use collision::prelude::*;
use collision::primitive::ConvexPolyhedron;
use genmesh::generators::{IndexedPolygon, SharedVertex, SphereUV};
use genmesh::Triangulate;
use rand::Rng;
use test::{black_box, Bencher};

#[bench]
fn test_polyhedron_10(bench: &mut Bencher) {
    test_polyhedron(bench, 10, false);
}

#[bench]
fn test_polyhedron_faces_10(bench: &mut Bencher) {
    test_polyhedron(bench, 10, true);
}

#[bench]
fn test_polyhedron_20(bench: &mut Bencher) {
    test_polyhedron(bench, 20, false);
}

#[bench]
fn test_polyhedron_faces_20(bench: &mut Bencher) {
    test_polyhedron(bench, 20, true);
}

#[bench]
fn test_polyhedron_30(bench: &mut Bencher) {
    test_polyhedron(bench, 30, false);
}

#[bench]
fn test_polyhedron_faces_30(bench: &mut Bencher) {
    test_polyhedron(bench, 30, true);
}

#[bench]
fn test_polyhedron_50(bench: &mut Bencher) {
    test_polyhedron(bench, 50, false);
}

#[bench]
fn test_polyhedron_faces_50(bench: &mut Bencher) {
    test_polyhedron(bench, 50, true);
}

#[bench]
fn test_polyhedron_500(bench: &mut Bencher) {
    test_polyhedron(bench, 500, false);
}

#[bench]
fn test_polyhedron_faces_500(bench: &mut Bencher) {
    test_polyhedron(bench, 500, true);
}

fn test_polyhedron(bench: &mut Bencher, n: usize, with_faces: bool) {
    let polyhedron = sphere(n, with_faces);
    let dirs = dirs(1000);
    let transform = Decomposed::<Vector3<f32>, Quaternion<f32>>::one();

    let mut i = 0;
    bench.iter(|| {
        black_box(polyhedron.support_point(&dirs[i % 1000], &transform));
        i += 1;
    });
}

fn dirs(n: usize) -> Vec<Vector3<f32>> {
    let mut rng = rand::thread_rng();
    (0..n)
        .map(|_| {
            Vector3::new(
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
            )
        })
        .collect::<Vec<_>>()
}

fn sphere(n: usize, with_faces: bool) -> ConvexPolyhedron<f32> {
    let gen = SphereUV::new(n, n);

    let vertices = gen
        .shared_vertex_iter()
        .map(|v| Point3::from(v.pos))
        .collect::<Vec<_>>();

    if with_faces {
        let faces = gen
            .indexed_polygon_iter()
            .triangulate()
            .map(|f| (f.x, f.y, f.z))
            .collect::<Vec<_>>();

        ConvexPolyhedron::new_with_faces(&vertices, &faces)
    } else {
        ConvexPolyhedron::new(&vertices)
    }
}
