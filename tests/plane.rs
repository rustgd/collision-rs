// Copyright 2013-2014 The CGMath Developers. For a full listing of the authors,
// refer to the Cargo.toml file at the top-level directory of this distribution.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#[macro_use]
extern crate approx;

extern crate cgmath;
extern crate collision;

use cgmath::*;
use collision::*;

#[test]
fn test_from_points() {
    assert_eq!(Plane::from_points(Point3::new(5.0f64, 0.0f64,  5.0f64),
                                  Point3::new(5.0f64, 5.0f64,  5.0f64),
                                  Point3::new(5.0f64, 0.0f64, -1.0f64)),
    	Some(Plane::from_abcd(-1.0f64, 0.0f64, 0.0f64, 5.0f64)));

    assert_eq!(Plane::from_points(Point3::new(0.0f64, 5.0f64, -5.0f64),
                                  Point3::new(0.0f64, 5.0f64,  0.0f64),
                                  Point3::new(0.0f64, 5.0f64,  5.0f64)),
        None);     // The points are parallel
}

#[test]
fn test_ray_intersection() {
    let p0 = Plane::from_abcd(1f64, 0f64, 0f64, -7f64);
    let r0: Ray3<f64> = Ray::new(Point3::new(2f64, 3f64, 4f64), Vector3::new(1f64, 1f64, 1f64).normalize());
    assert_eq!((p0, r0).intersection(), Some(Point3::new(7f64, 8f64, 9f64)));

    let p1 = Plane::from_points(Point3::new(5f64, 0f64,  5f64),
                                Point3::new(5f64, 5f64,  5f64),
                                Point3::new(5f64, 0f64, -1f64)).unwrap();
    let r1: Ray3<f64> = Ray::new(Point3::new(0f64, 0f64, 0f64), Vector3::new(-1f64, 0f64, 0f64).normalize());
    assert_eq!((p1, r1).intersection(), None); // r1 points away from p1
}

#[test]
fn test_plane2_intersection() {
    let p0 = Plane::new(Vector3::unit_x(), 1.0f64);
    let p1 = Plane::new(Vector3::unit_y(), 2.0f64);
    let ray = (p0, p1).intersection();
    assert!(ray.is_some());

    let ray = ray.unwrap();
    assert_ulps_eq!(ray.origin.x, &1.0f64);
    assert_ulps_eq!(ray.origin.y, &2.0f64);
    assert_ulps_eq!(ray.origin.z, &0.0f64);
    assert_ulps_eq!(ray.direction.x, &0.0f64);
    assert_ulps_eq!(ray.direction.y, &0.0f64);
    assert_ulps_eq!(ray.direction.z, &1.0f64);

    let p0 = Plane::new(Vector3::unit_y(), 1.0f64);
    let p1 = Plane::new(Vector3::unit_y(), 2.0f64);
    let ray = (p0, p1).intersection();
    assert!(ray.is_none());
}

#[test]
fn test_plane3_intersection() {
    let p0 = Plane::new(Vector3::unit_x(), 1.0f64);
    let p1 = Plane::new(Vector3::unit_y(), 2.0f64);
    let p2 = Plane::new(Vector3::unit_z(), 3.0f64);
    let point = (p0, p1, p2).intersection();
    assert!(point.is_some());

    let point = point.unwrap();
    assert_ulps_eq!(point.x, &1.0f64);
    assert_ulps_eq!(point.y, &2.0f64);
    assert_ulps_eq!(point.z, &3.0f64);

    let p0 = Plane::new(Vector3::unit_y(), 1.0f64);
    let p1 = Plane::new(Vector3::unit_y(), 2.0f64);
    let p2 = Plane::new(Vector3::unit_z(), 3.0f64);
    let point = (p0, p1, p2).intersection();
    assert!(point.is_none());
}
