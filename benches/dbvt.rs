#![feature(test)]

extern crate test;

use cgmath::prelude::*;
use cgmath::{Point2, Vector2};
use collision::dbvt::*;
use collision::prelude::*;
use collision::{Aabb2, Ray2};
use rand::Rng;
use test::Bencher;

#[derive(Debug, Clone)]
struct Value {
    pub id: u32,
    pub aabb: Aabb2<f32>,
    fat_aabb: Aabb2<f32>,
}

impl Value {
    pub fn new(id: u32, aabb: Aabb2<f32>) -> Self {
        Self {
            id,
            fat_aabb: aabb.add_margin(Vector2::new(0., 0.)),
            aabb,
        }
    }
}

impl TreeValue for Value {
    type Bound = Aabb2<f32>;

    fn bound(&self) -> &Aabb2<f32> {
        &self.aabb
    }

    fn get_bound_with_margin(&self) -> Aabb2<f32> {
        self.fat_aabb
    }
}

#[bench]
fn benchmark_insert(b: &mut Bencher) {
    let mut rng = rand::thread_rng();
    let mut tree = DynamicBoundingVolumeTree::<Value>::new();
    let mut i = 0;
    b.iter(|| {
        let offset = rng.gen_range(0.0..100.0);
        tree.insert(Value::new(
            i,
            aabb2(offset + 2., offset + 2., offset + 4., offset + 4.),
        ));
        i += 1;
    });
}

#[bench]
fn benchmark_do_refit(b: &mut Bencher) {
    let mut rng = rand::thread_rng();
    let mut tree = DynamicBoundingVolumeTree::<Value>::new();
    let mut i = 0;
    b.iter(|| {
        let offset = rng.gen_range(0.0..100.0);
        tree.insert(Value::new(
            i,
            aabb2(offset + 2., offset + 2., offset + 4., offset + 4.),
        ));
        tree.do_refit();
        i += 1;
    });
}

#[bench]
fn benchmark_query(b: &mut Bencher) {
    let mut rng = rand::thread_rng();
    let mut tree = DynamicBoundingVolumeTree::<Value>::new();
    for i in 0..100000 {
        let neg_y = if rng.gen::<bool>() { -1. } else { 1. };
        let neg_x = if rng.gen::<bool>() { -1. } else { 1. };
        let offset_x = neg_x * rng.gen_range(9000.0..10000.0);
        let offset_y = neg_y * rng.gen_range(9000.0..10000.0);
        tree.insert(Value::new(
            i,
            aabb2(offset_x + 2., offset_y + 2., offset_x + 4., offset_y + 4.),
        ));
        tree.tick();
    }

    let rays: Vec<Ray2<f32>> = (0..1000)
        .map(|_| {
            let p = Point2::new(
                rng.gen_range(-10000.0..10000.0),
                rng.gen_range(-10000.0..10000.0),
            );
            let d = Vector2::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)).normalize();
            Ray2::new(p, d)
        })
        .collect();

    let mut visitors: Vec<DiscreteVisitor<Ray2<f32>, Value>> = rays
        .iter()
        .map(|ray| DiscreteVisitor::<Ray2<f32>, Value>::new(ray))
        .collect();

    let mut i = 0;

    b.iter(|| {
        tree.query(&mut visitors[i % 1000]).len();
        i += 1;
    });
}

#[bench]
fn benchmark_ray_closest_query(b: &mut Bencher) {
    let mut rng = rand::thread_rng();
    let mut tree = DynamicBoundingVolumeTree::<Value>::new();
    for i in 0..100000 {
        let neg_y = if rng.gen::<bool>() { -1. } else { 1. };
        let neg_x = if rng.gen::<bool>() { -1. } else { 1. };
        let offset_x = neg_x * rng.gen_range(9000.0..10000.0);
        let offset_y = neg_y * rng.gen_range(9000.0..10000.0);
        tree.insert(Value::new(
            i,
            aabb2(offset_x + 2., offset_y + 2., offset_x + 4., offset_y + 4.),
        ));
        tree.tick();
    }

    let rays: Vec<Ray2<f32>> = (0..1000)
        .map(|_| {
            let p = Point2::new(
                rng.gen_range(-10000.0..10000.0),
                rng.gen_range(-10000.0..10000.0),
            );
            let d = Vector2::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)).normalize();
            Ray2::new(p, d)
        })
        .collect();

    let mut i = 0;

    b.iter(|| {
        query_ray_closest(&tree, rays[i % 1000]);
        i += 1;
    });
}

fn aabb2(minx: f32, miny: f32, maxx: f32, maxy: f32) -> Aabb2<f32> {
    Aabb2::new(Point2::new(minx, miny), Point2::new(maxx, maxy))
}
