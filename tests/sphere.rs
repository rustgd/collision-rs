// Copyright 2014 The CGMath Developers. For a full listing of the authors,
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
fn test_intersection() {
    let sphere = Sphere {
        center: Point3::new(0f64, 0f64, 0f64),
        radius: 1f64,
    };
    let r0 = Ray::new(
        Point3::new(0f64, 0f64, 5f64),
        Vector3::new(0f64, 0f64, -5f64).normalize(),
    );
    let r1 = Ray::new(
        Point3::new(1f64.cos(), 0f64, 5f64),
        Vector3::new(0f64, 0f64, -5f64).normalize(),
    );
    let r2 = Ray::new(
        Point3::new(1f64, 0f64, 5f64),
        Vector3::new(0f64, 0f64, -5f64).normalize(),
    );
    let r3 = Ray::new(
        Point3::new(2f64, 0f64, 5f64),
        Vector3::new(0f64, 0f64, -5f64).normalize(),
    );
    assert_eq!(
        sphere.intersection(&r0),
        Some(Point3::new(0f64, 0f64, 1f64))
    );
    assert!(sphere.intersects(&r0));
    assert_ulps_eq!(
        sphere.intersection(&r1).unwrap(),
        &Point3::new(1f64.cos(), 0f64, 1f64.sin())
    );
    assert!(sphere.intersects(&r1));
    assert_eq!(
        sphere.intersection(&r2),
        Some(Point3::new(1f64, 0f64, 0f64))
    );
    assert!(sphere.intersects(&r2));
    assert_eq!(sphere.intersection(&r3), None);
    assert!(!sphere.intersects(&r3));
}

#[test]
fn test_bound() {
    let point = Point3::new(1f32, 2.0, 3.0);
    let sphere = Sphere {
        center: point,
        radius: 1.0,
    };
    let normal = vec3(0f32, 0.0, 1.0);

    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point, normal)),
        Relation::Cross
    );
    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point + normal * -3.0, normal)),
        Relation::In
    );
    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point + normal * 3.0, normal)),
        Relation::Out
    );
}
